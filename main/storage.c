#include <stdio.h>
#include <string.h>
#include <dirent.h>
#include <sys/stat.h>
#include <unistd.h>

#include "esp_log.h"
#include "esp_timer.h"
#include "esp_littlefs.h"
#include "nvs.h"

#include "storage.h"


// ============================================================
// CONFIGURATION
// ============================================================

#define STORAGE_BASE        "/storage"
#define STORAGE_PARTITION   "storage"
#define TMP_PATH            STORAGE_BASE "/tmp.bin"

// Free space kept in reserve on top of the incoming file so LittleFS
// always has room for its own metadata blocks.
#define SPACE_MARGIN        (16 * 1024)

#define WRITE_BUFFER_SIZE   2048

// Song cache index. Stored in NVS (wear-levelled, atomic writes) rather
// than in a file on the song partition.
#define LRU_MAX_ENTRIES     64
#define NVS_NAMESPACE       "song_cache"
#define NVS_KEY_LRU         "lru"


static const char *TAG = "storage";


// ============================================================
// CRC-32 (same as zlib.crc32 / binascii.crc32)
// ============================================================

static uint32_t s_crc_table[256];


static void crc32_init(void)
{
    for (uint32_t i = 0; i < 256; i++)
    {
        uint32_t c = i;

        for (int k = 0; k < 8; k++)
            c = (c & 1) ? (0xEDB88320u ^ (c >> 1)) : (c >> 1);

        s_crc_table[i] = c;
    }
}


static uint32_t crc32_update(uint32_t crc, const uint8_t *data, size_t len)
{
    crc = ~crc;

    while (len--)
        crc = s_crc_table[(crc ^ *data++) & 0xFF] ^ (crc >> 8);

    return ~crc;
}


// ============================================================
// LEAST-RECENTLY-USED SONG CACHE
// ============================================================

typedef struct __attribute__((packed))
{
    char track_id[TRACK_ID_LEN];
    uint32_t last_used;

} lru_entry_t;


static lru_entry_t s_lru[LRU_MAX_ENTRIES];
static int s_lru_count = 0;
static uint32_t s_lru_counter = 0;


static int lru_find(const char *track_id)
{
    for (int i = 0; i < s_lru_count; i++)
    {
        if (memcmp(s_lru[i].track_id, track_id, TRACK_ID_LEN) == 0)
            return i;
    }

    return -1;
}


static int lru_oldest(void)
{
    int oldest = -1;

    for (int i = 0; i < s_lru_count; i++)
    {
        if (oldest < 0 || s_lru[i].last_used < s_lru[oldest].last_used)
            oldest = i;
    }

    return oldest;
}


static void lru_remove(int index)
{
    s_lru[index] = s_lru[s_lru_count - 1];
    s_lru_count--;
}


static void lru_save(void)
{
    nvs_handle_t nvs;

    if (nvs_open(NVS_NAMESPACE, NVS_READWRITE, &nvs) != ESP_OK)
    {
        ESP_LOGE(TAG, "Could not open NVS to save cache index");
        return;
    }

    esp_err_t err;

    if (s_lru_count == 0)
    {
        err = nvs_erase_key(nvs, NVS_KEY_LRU);

        if (err == ESP_ERR_NVS_NOT_FOUND)
            err = ESP_OK;
    }
    else
    {
        err = nvs_set_blob(
            nvs,
            NVS_KEY_LRU,
            s_lru,
            s_lru_count * sizeof(lru_entry_t)
        );
    }

    if (err == ESP_OK)
        err = nvs_commit(nvs);

    if (err != ESP_OK)
        ESP_LOGE(TAG, "Saving cache index failed: %s", esp_err_to_name(err));

    nvs_close(nvs);
}


static void delete_track_file(const char *track_id)
{
    char path[48];
    storage_track_path(track_id, path, sizeof(path));

    if (unlink(path) == 0)
        ESP_LOGI(TAG, "Deleted %.22s from cache", track_id);
}


static void evict_entry(int index)
{
    ESP_LOGI(TAG, "Evicting least recently used track %.22s", s_lru[index].track_id);

    delete_track_file(s_lru[index].track_id);
    lru_remove(index);
}


// Rebuilds the index from the files that actually exist on the
// partition, keeping last-used values saved in NVS where possible.
static void lru_rebuild(void)
{
    static lru_entry_t saved[LRU_MAX_ENTRIES];
    int saved_count = 0;

    nvs_handle_t nvs;

    if (nvs_open(NVS_NAMESPACE, NVS_READONLY, &nvs) == ESP_OK)
    {
        size_t size = sizeof(saved);

        if (nvs_get_blob(nvs, NVS_KEY_LRU, saved, &size) == ESP_OK)
            saved_count = size / sizeof(lru_entry_t);

        nvs_close(nvs);
    }

    s_lru_count = 0;
    s_lru_counter = 0;

    bool found_tmp = false;

    DIR *dir = opendir(STORAGE_BASE);

    if (dir == NULL)
    {
        ESP_LOGE(TAG, "Could not list %s", STORAGE_BASE);
        return;
    }

    struct dirent *entry;

    while ((entry = readdir(dir)) != NULL)
    {
        const char *name = entry->d_name;

        if (strcmp(name, "tmp.bin") == 0)
        {
            found_tmp = true;
            continue;
        }

        bool is_track_file =
            strlen(name) == TRACK_ID_LEN + 4 &&
            strcmp(name + TRACK_ID_LEN, ".bin") == 0 &&
            track_id_is_valid(name);

        if (!is_track_file)
        {
            ESP_LOGW(TAG, "Ignoring unexpected file %s", name);
            continue;
        }

        if (s_lru_count >= LRU_MAX_ENTRIES)
        {
            ESP_LOGW(TAG, "Cache index full, ignoring %s", name);
            continue;
        }

        lru_entry_t *e = &s_lru[s_lru_count++];
        memcpy(e->track_id, name, TRACK_ID_LEN);
        e->last_used = 0;

        for (int i = 0; i < saved_count; i++)
        {
            if (memcmp(saved[i].track_id, name, TRACK_ID_LEN) == 0)
            {
                e->last_used = saved[i].last_used;
                break;
            }
        }

        if (e->last_used > s_lru_counter)
            s_lru_counter = e->last_used;
    }

    closedir(dir);

    // A leftover temp file means a transfer was interrupted by a reset.
    if (found_tmp)
        unlink(TMP_PATH);

    lru_save();

    ESP_LOGI(TAG, "%d song(s) cached", s_lru_count);
}


// Deletes least recently used songs until `needed` bytes fit.
static uint8_t make_space(uint32_t needed)
{
    for (;;)
    {
        size_t total = 0;
        size_t used = 0;

        if (esp_littlefs_info(STORAGE_PARTITION, &total, &used) != ESP_OK)
            return PROTO_ERR_FS_WRITE;

        if (total - used >= (size_t)needed + SPACE_MARGIN)
            return PROTO_ERR_NONE;

        int victim = lru_oldest();

        if (victim < 0)
            return PROTO_ERR_NO_SPACE;

        evict_entry(victim);
        lru_save();
    }
}


// ============================================================
// PUBLIC: GENERAL
// ============================================================

esp_err_t storage_init(void)
{
    crc32_init();

    esp_vfs_littlefs_conf_t config = {
        .base_path = STORAGE_BASE,
        .partition_label = STORAGE_PARTITION,
        // The partition still holds the old SPIFFS image on first boot,
        // so it is formatted automatically.
        .format_if_mount_failed = true,
        .dont_mount = false,
    };

    esp_err_t result = esp_vfs_littlefs_register(&config);

    if (result != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to mount LittleFS: %s", esp_err_to_name(result));
        return result;
    }

    size_t total = 0;
    size_t used = 0;

    esp_littlefs_info(STORAGE_PARTITION, &total, &used);

    ESP_LOGI(
        TAG,
        "LittleFS mounted: %u / %u bytes used",
        (unsigned int)used,
        (unsigned int)total
    );

    lru_rebuild();

    return ESP_OK;
}


void storage_track_path(const char *track_id, char *out, size_t out_len)
{
    snprintf(out, out_len, STORAGE_BASE "/%.22s.bin", track_id);
}


bool storage_has_track(const char *track_id)
{
    if (!track_id_is_valid(track_id))
        return false;

    char path[48];
    storage_track_path(track_id, path, sizeof(path));

    struct stat st;
    return stat(path, &st) == 0 && st.st_size > 0;
}


void storage_touch(const char *track_id)
{
    int index = lru_find(track_id);

    if (index < 0)
    {
        if (s_lru_count >= LRU_MAX_ENTRIES)
            evict_entry(lru_oldest());

        index = s_lru_count++;
        memcpy(s_lru[index].track_id, track_id, TRACK_ID_LEN);
    }

    s_lru[index].last_used = ++s_lru_counter;
    lru_save();
}


// ============================================================
// PUBLIC: FILE TRANSFER
// ============================================================

static struct
{
    bool active;
    FILE *file;
    char track_id[TRACK_ID_LEN];

    uint32_t total_size;
    uint32_t expected_crc;
    uint32_t crc;
    uint32_t received;

    int64_t last_activity_us;

} s_xfer;


void storage_xfer_abort(void)
{
    if (s_xfer.file != NULL)
    {
        fclose(s_xfer.file);
        s_xfer.file = NULL;
    }

    if (s_xfer.active)
    {
        unlink(TMP_PATH);
        s_xfer.active = false;
    }
}


uint8_t storage_xfer_begin(const char *track_id, uint32_t total_size, uint32_t crc32)
{
    storage_xfer_abort();

    if (!track_id_is_valid(track_id))
        return PROTO_ERR_BAD_MESSAGE;

    if (total_size == 0 || total_size % AUDIO_FRAME_SIZE != 0)
        return PROTO_ERR_BAD_SIZE;

    // Replace any existing copy of this track.
    int existing = lru_find(track_id);

    if (existing >= 0)
    {
        lru_remove(existing);
        lru_save();
    }

    delete_track_file(track_id);
    unlink(TMP_PATH);

    size_t total = 0;
    size_t used = 0;

    esp_littlefs_info(STORAGE_PARTITION, &total, &used);

    if ((size_t)total_size + SPACE_MARGIN > total)
        return PROTO_ERR_TOO_LARGE;

    uint8_t err = make_space(total_size);

    if (err != PROTO_ERR_NONE)
        return err;

    s_xfer.file = fopen(TMP_PATH, "wb");

    if (s_xfer.file == NULL)
        return PROTO_ERR_FS_WRITE;

    setvbuf(s_xfer.file, NULL, _IOFBF, WRITE_BUFFER_SIZE);

    memcpy(s_xfer.track_id, track_id, TRACK_ID_LEN);
    s_xfer.total_size = total_size;
    s_xfer.expected_crc = crc32;
    s_xfer.crc = 0;
    s_xfer.received = 0;
    s_xfer.last_activity_us = esp_timer_get_time();
    s_xfer.active = true;

    ESP_LOGI(
        TAG,
        "Receiving %.22s (%lu bytes)",
        track_id,
        (unsigned long)total_size
    );

    return PROTO_ERR_NONE;
}


uint8_t storage_xfer_chunk(uint32_t offset, const uint8_t *data, size_t len)
{
    if (!s_xfer.active)
        return PROTO_ERR_NOT_RECEIVING;

    if (offset != s_xfer.received)
    {
        ESP_LOGE(
            TAG,
            "Chunk offset %lu, expected %lu",
            (unsigned long)offset,
            (unsigned long)s_xfer.received
        );

        storage_xfer_abort();
        return PROTO_ERR_BAD_OFFSET;
    }

    if (s_xfer.received + len > s_xfer.total_size)
    {
        storage_xfer_abort();
        return PROTO_ERR_SIZE_MISMATCH;
    }

    if (fwrite(data, 1, len, s_xfer.file) != len)
    {
        ESP_LOGE(TAG, "Write to %s failed", TMP_PATH);
        storage_xfer_abort();
        return PROTO_ERR_FS_WRITE;
    }

    s_xfer.crc = crc32_update(s_xfer.crc, data, len);
    s_xfer.received += len;
    s_xfer.last_activity_us = esp_timer_get_time();

    return PROTO_ERR_NONE;
}


uint8_t storage_xfer_end(void)
{
    if (!s_xfer.active)
        return PROTO_ERR_NOT_RECEIVING;

    int close_result = fclose(s_xfer.file);
    s_xfer.file = NULL;

    uint8_t err = PROTO_ERR_NONE;

    if (close_result != 0)
    {
        err = PROTO_ERR_FS_WRITE;
    }
    else if (s_xfer.received != s_xfer.total_size)
    {
        ESP_LOGE(
            TAG,
            "Size mismatch: got %lu, expected %lu",
            (unsigned long)s_xfer.received,
            (unsigned long)s_xfer.total_size
        );

        err = PROTO_ERR_SIZE_MISMATCH;
    }
    else if (s_xfer.crc != s_xfer.expected_crc)
    {
        ESP_LOGE(
            TAG,
            "CRC mismatch: got %08lx, expected %08lx",
            (unsigned long)s_xfer.crc,
            (unsigned long)s_xfer.expected_crc
        );

        err = PROTO_ERR_CRC_MISMATCH;
    }
    else
    {
        char path[48];
        storage_track_path(s_xfer.track_id, path, sizeof(path));

        if (rename(TMP_PATH, path) != 0)
            err = PROTO_ERR_RENAME;
    }

    if (err != PROTO_ERR_NONE)
    {
        unlink(TMP_PATH);
        s_xfer.active = false;
        return err;
    }

    s_xfer.active = false;
    storage_touch(s_xfer.track_id);

    ESP_LOGI(TAG, "Stored %.22s", s_xfer.track_id);

    return PROTO_ERR_NONE;
}


bool storage_xfer_active(void)
{
    return s_xfer.active;
}


void storage_xfer_track(char out[TRACK_ID_LEN])
{
    memcpy(out, s_xfer.track_id, TRACK_ID_LEN);
}


uint32_t storage_xfer_received(void)
{
    return s_xfer.received;
}


uint32_t storage_xfer_total(void)
{
    return s_xfer.total_size;
}


int64_t storage_xfer_idle_us(void)
{
    return esp_timer_get_time() - s_xfer.last_activity_us;
}
