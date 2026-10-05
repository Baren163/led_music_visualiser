#include <stdio.h>
#include <string.h>
#include <stdlib.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"

#include "esp_log.h"
#include "esp_timer.h"
#include "nvs.h"

#include "control.h"
#include "renderer.h"
#include "storage.h"
#include "ble.h"


// ============================================================
// CONFIGURATION
// ============================================================

#define CONTROL_TASK_PRIORITY       5
#define CONTROL_TASK_STACK          6144
#define CONTROL_QUEUE_LENGTH        8

// How long the BLE host task may block when the queue is full before
// the write is rejected (the laptop then retries the transfer).
#define CONTROL_QUEUE_TIMEOUT_MS    500

#define CONTROL_TICK_MS             500

// Turn the strip off if the laptop stops sending STATE.
#define STATE_TIMEOUT_MS            15000

// Abandon a transfer if no chunk arrives for this long.
#define TRANSFER_TIMEOUT_MS         5000

// Only seek when the renderer is further than this from the target.
// Smaller corrections every second would cause visible jitter.
#define DRIFT_THRESHOLD_FRAMES      2

#define NVS_NAMESPACE               "visualiser"
#define NVS_KEY_OFFSET              "offset_ms"


static const char *TAG = "control";


typedef struct
{
    uint8_t kind;           // ctrl_kind_t
    uint16_t len;
    uint8_t data[PROTO_MAX_WRITE_LEN];

} ctrl_msg_t;


static QueueHandle_t s_queue;

// STATUS is written by the control task and read by the BLE host task.
static SemaphoreHandle_t s_status_lock;
static status_msg_t s_status;

static int16_t s_offset_ms = 0;

static bool s_have_seq = false;
static uint16_t s_last_seq = 0;

static bool s_state_active = false;
static int64_t s_last_state_us = 0;

// Last track the laptop asked for, only used to avoid repeating logs.
static char s_last_requested[TRACK_ID_LEN];


// ============================================================
// STATUS HELPERS
// ============================================================

static void status_lock(void)
{
    xSemaphoreTake(s_status_lock, portMAX_DELAY);
}


static void status_unlock(void)
{
    xSemaphoreGive(s_status_lock);
}


static void publish_status(void)
{
    status_msg_t copy;

    control_get_status(&copy);
    ble_notify_status(&copy);
}


static void set_transfer_status(transfer_state_t state, uint8_t error, uint32_t bytes)
{
    status_lock();

    s_status.transfer_state = state;
    s_status.bytes_received = bytes;

    if (error != PROTO_ERR_NONE || state == TRANSFER_RECEIVING)
        s_status.error_code = error;

    status_unlock();
}


static void refresh_loaded_track(void)
{
    char loaded[TRACK_ID_LEN];
    renderer_get_loaded_track(loaded);

    status_lock();
    memcpy(s_status.track_id, loaded, TRACK_ID_LEN);
    status_unlock();
}


static void fail_transfer(uint8_t error)
{
    ESP_LOGE(TAG, "Transfer failed, error %u", error);

    storage_xfer_abort();
    set_transfer_status(TRANSFER_ERROR, error, 0);
    renderer_idle();
    refresh_loaded_track();
    publish_status();
}


// ============================================================
// NVS OFFSET
// ============================================================

static void load_offset(void)
{
    nvs_handle_t nvs;

    if (nvs_open(NVS_NAMESPACE, NVS_READONLY, &nvs) == ESP_OK)
    {
        nvs_get_i16(nvs, NVS_KEY_OFFSET, &s_offset_ms);
        nvs_close(nvs);
    }

    ESP_LOGI(TAG, "Latency offset: %d ms", s_offset_ms);
}


static void save_offset(void)
{
    nvs_handle_t nvs;

    if (nvs_open(NVS_NAMESPACE, NVS_READWRITE, &nvs) != ESP_OK)
        return;

    nvs_set_i16(nvs, NVS_KEY_OFFSET, s_offset_ms);
    nvs_commit(nvs);
    nvs_close(nvs);
}


// ============================================================
// STATE HANDLING
// ============================================================

static void log_missing_track(const msg_state_t *m)
{
    if (memcmp(s_last_requested, m->track_id, TRACK_ID_LEN) == 0)
        return;

    if (m->flags & STATE_FLAG_FILE_AVAILABLE)
        ESP_LOGI(TAG, "%.22s not cached, waiting for laptop to send it", m->track_id);
    else
        ESP_LOGI(TAG, "No visualisation for %.22s, showing idle", m->track_id);
}


static void apply_state(const msg_state_t *m)
{
    bool playing = (m->flags & STATE_FLAG_PLAYING) != 0;

    int32_t position = (int32_t)m->position_ms + s_offset_ms;

    if (position < 0)
        position = 0;

    uint32_t target_frame = (uint32_t)position / FRAME_PERIOD_MS;

    char loaded[TRACK_ID_LEN];
    renderer_get_loaded_track(loaded);

    if (track_id_is_empty(m->track_id) || !track_id_is_valid(m->track_id))
    {
        // Nothing playing, or something without a track ID
        // (podcast, local file).
        if (playing)
            renderer_idle();
        else
            renderer_off();
    }
    else if (memcmp(loaded, m->track_id, TRACK_ID_LEN) != 0)
    {
        // ----------------------------------------------------
        // Different track
        // ----------------------------------------------------

        if (storage_has_track(m->track_id) &&
            renderer_open(m->track_id, target_frame, playing))
        {
            // Touch after opening so eviction can never hit the file
            // the renderer had open before.
            storage_touch(m->track_id);
        }
        else
        {
            log_missing_track(m);
            renderer_idle();
        }
    }
    else
    {
        // ----------------------------------------------------
        // Same track
        // ----------------------------------------------------

        renderer_mode_t mode = renderer_get_mode();

        if (!playing)
        {
            if (mode == RENDERER_PLAYING)
            {
                ESP_LOGI(TAG, "Paused");
                renderer_pause();
            }
        }
        else if (mode == RENDERER_PAUSED || mode == RENDERER_ENDED)
        {
            if (mode == RENDERER_PAUSED)
                ESP_LOGI(TAG, "Resuming at frame %lu", (unsigned long)target_frame);

            renderer_resume(target_frame);
        }
        else if (mode == RENDERER_PLAYING)
        {
            int32_t drift = (int32_t)target_frame - (int32_t)renderer_get_frame();

            if (abs(drift) > DRIFT_THRESHOLD_FRAMES)
            {
                ESP_LOGI(
                    TAG,
                    "Drift %ld frames, seeking to %lu",
                    (long)drift,
                    (unsigned long)target_frame
                );

                renderer_seek(target_frame);
            }
        }
    }

    memcpy(s_last_requested, m->track_id, TRACK_ID_LEN);
    refresh_loaded_track();
}


static void handle_state(const msg_state_t *m)
{
    // Drop stale or out-of-order messages (seq wraps at 65536).
    if (s_have_seq && (int16_t)(m->seq - s_last_seq) <= 0)
    {
        ESP_LOGD(TAG, "Dropping stale STATE seq %u", m->seq);
        return;
    }

    s_have_seq = true;
    s_last_seq = m->seq;
    s_state_active = true;
    s_last_state_us = esp_timer_get_time();

    bool keep_transfer = false;

    // A transfer is in progress: keep showing it unless the laptop
    // has moved on to a different track.
    if (storage_xfer_active())
    {
        char xfer_track[TRACK_ID_LEN];
        storage_xfer_track(xfer_track);

        keep_transfer = memcmp(xfer_track, m->track_id, TRACK_ID_LEN) == 0;

        if (!keep_transfer)
        {
            ESP_LOGW(TAG, "Track changed during transfer, aborting it");
            storage_xfer_abort();
            set_transfer_status(TRANSFER_ERROR, PROTO_ERR_ABORTED, 0);
        }
    }

    if (!keep_transfer)
        apply_state(m);

    status_lock();
    s_status.ack_seq = m->seq;
    status_unlock();

    publish_status();
}


static void handle_set_offset(const msg_set_offset_t *m)
{
    s_offset_ms = m->offset_ms;
    save_offset();

    ESP_LOGI(TAG, "Latency offset set to %d ms", s_offset_ms);

    status_lock();
    s_status.offset_ms = s_offset_ms;
    status_unlock();

    publish_status();
}


// ============================================================
// FILE TRANSFER HANDLING
// ============================================================

static void handle_file_start(const msg_file_start_t *m)
{
    // Close the song file before anything is deleted.
    renderer_show_transfer();
    refresh_loaded_track();

    uint8_t err = storage_xfer_begin(m->track_id, m->total_size, m->crc32);

    if (err != PROTO_ERR_NONE)
    {
        fail_transfer(err);
        return;
    }

    set_transfer_status(TRANSFER_RECEIVING, PROTO_ERR_NONE, 0);
    publish_status();
}


static void handle_file_chunk(const uint8_t *data, uint16_t len)
{
    if (!storage_xfer_active())
    {
        // Only report the first rejected chunk, not every one after it.
        status_msg_t status;
        control_get_status(&status);

        if (status.transfer_state != TRANSFER_ERROR)
            fail_transfer(PROTO_ERR_NOT_RECEIVING);

        return;
    }

    uint32_t offset;
    memcpy(&offset, data + 1, sizeof(offset));

    uint8_t err = storage_xfer_chunk(
        offset,
        data + FILE_CHUNK_HEADER_LEN,
        len - FILE_CHUNK_HEADER_LEN
    );

    if (err != PROTO_ERR_NONE)
    {
        fail_transfer(err);
        return;
    }

    uint32_t received = storage_xfer_received();
    uint32_t total = storage_xfer_total();

    renderer_set_progress((uint8_t)(((uint64_t)received * 100) / total));

    status_lock();
    s_status.bytes_received = received;
    status_unlock();
}


static void handle_file_end(void)
{
    uint32_t received = storage_xfer_received();
    uint8_t err = storage_xfer_end();

    if (err != PROTO_ERR_NONE)
    {
        fail_transfer(err);
        return;
    }

    // The next STATE message opens the new file.
    renderer_idle();
    refresh_loaded_track();

    set_transfer_status(TRANSFER_OK, PROTO_ERR_NONE, received);
    publish_status();
}


// ============================================================
// DISPATCH
// ============================================================

static void handle_message(const ctrl_msg_t *msg)
{
    switch (msg->kind)
    {
    case CTRL_SRC_STATE_CHR:
        if (msg->data[0] == MSG_STATE)
            handle_state((const msg_state_t *)msg->data);
        else
            handle_set_offset((const msg_set_offset_t *)msg->data);
        break;

    case CTRL_SRC_FILE_CHR:
        switch (msg->data[0])
        {
        case MSG_FILE_START:
            handle_file_start((const msg_file_start_t *)msg->data);
            break;

        case MSG_FILE_CHUNK:
            handle_file_chunk(msg->data, msg->len);
            break;

        case MSG_FILE_END:
            handle_file_end();
            break;
        }
        break;

    case CTRL_EVT_CONNECTED:
        ESP_LOGI(TAG, "Laptop connected");
        // A restarted laptop program starts its seq from zero again.
        s_have_seq = false;
        break;

    case CTRL_EVT_DISCONNECTED:
        ESP_LOGI(TAG, "Laptop disconnected");

        if (storage_xfer_active())
            fail_transfer(PROTO_ERR_ABORTED);
        break;
    }
}


static void check_timeouts(void)
{
    if (storage_xfer_active() &&
        storage_xfer_idle_us() > (int64_t)TRANSFER_TIMEOUT_MS * 1000)
    {
        ESP_LOGW(TAG, "Transfer timed out");
        fail_transfer(PROTO_ERR_TIMEOUT);
    }

    if (s_state_active &&
        esp_timer_get_time() - s_last_state_us > (int64_t)STATE_TIMEOUT_MS * 1000)
    {
        ESP_LOGW(TAG, "No STATE from laptop for %d s, turning strip off", STATE_TIMEOUT_MS / 1000);

        s_state_active = false;
        renderer_off();
        refresh_loaded_track();
        publish_status();
    }
}


static void control_task(void *arg)
{
    // Large message buffer: keep it off the task stack.
    static ctrl_msg_t msg;

    for (;;)
    {
        if (xQueueReceive(s_queue, &msg, pdMS_TO_TICKS(CONTROL_TICK_MS)) == pdTRUE)
            handle_message(&msg);

        check_timeouts();
    }
}


// ============================================================
// PUBLIC API
// ============================================================

static bool message_is_valid(ctrl_kind_t kind, const uint8_t *data, uint16_t len)
{
    if (len < 1)
        return false;

    if (kind == CTRL_SRC_STATE_CHR)
    {
        switch (data[0])
        {
        case MSG_STATE:      return len == sizeof(msg_state_t);
        case MSG_SET_OFFSET: return len == sizeof(msg_set_offset_t);
        default:             return false;
        }
    }

    if (kind == CTRL_SRC_FILE_CHR)
    {
        switch (data[0])
        {
        case MSG_FILE_START: return len == sizeof(msg_file_start_t);
        case MSG_FILE_CHUNK: return len > FILE_CHUNK_HEADER_LEN && len <= PROTO_MAX_WRITE_LEN;
        case MSG_FILE_END:   return len == 1;
        default:             return false;
        }
    }

    return false;
}


ctrl_post_result_t control_post_write(ctrl_kind_t kind, const uint8_t *data, uint16_t len)
{
    if (!message_is_valid(kind, data, len))
        return CTRL_POST_INVALID;

    ctrl_msg_t msg = {
        .kind = kind,
        .len = len,
    };

    memcpy(msg.data, data, len);

    // Blocking here delays the write response, which throttles the
    // laptop if flash writes ever fall behind.
    if (xQueueSend(s_queue, &msg, pdMS_TO_TICKS(CONTROL_QUEUE_TIMEOUT_MS)) != pdTRUE)
    {
        ESP_LOGW(TAG, "Control queue full, rejecting write");
        return CTRL_POST_BUSY;
    }

    return CTRL_POST_OK;
}


void control_post_event(ctrl_kind_t kind)
{
    ctrl_msg_t msg = {
        .kind = kind,
        .len = 0,
    };

    if (xQueueSend(s_queue, &msg, pdMS_TO_TICKS(CONTROL_QUEUE_TIMEOUT_MS)) != pdTRUE)
        ESP_LOGW(TAG, "Control queue full, dropped event %d", kind);
}


void control_get_status(status_msg_t *out)
{
    status_lock();
    *out = s_status;
    status_unlock();
}


void control_init(void)
{
    s_status_lock = xSemaphoreCreateMutex();
    s_queue = xQueueCreate(CONTROL_QUEUE_LENGTH, sizeof(ctrl_msg_t));

    configASSERT(s_status_lock != NULL);
    configASSERT(s_queue != NULL);

    load_offset();

    memset(&s_status, 0, sizeof(s_status));
    s_status.transfer_state = TRANSFER_IDLE;
    s_status.offset_ms = s_offset_ms;

    xTaskCreate(
        control_task,
        "control",
        CONTROL_TASK_STACK,
        NULL,
        CONTROL_TASK_PRIORITY,
        NULL
    );
}
