#pragma once

// ============================================================
// LAPTOP <-> ESP32 BLE PROTOCOL
// ============================================================
//
// All messages are packed and little-endian. The first byte of
// every message written by the laptop is its type.
//
// GATT layout (one custom service, three characteristics):
//
//   Service  a3c87500-8ed3-4bdf-8a39-a01bebede295
//   STATE    a3c87501-...   laptop -> ESP (write)        STATE, SET_OFFSET
//   FILE     a3c87502-...   laptop -> ESP (write)        FILE_START/CHUNK/END
//   STATUS   a3c87503-...   ESP -> laptop (read+notify)  status_msg_t
//
// Keep this file in sync with laptop/visualiser_link.py.
// ============================================================

#include <stdint.h>
#include <stdbool.h>


#define TRACK_ID_LEN            22      // Spotify base62 track ID

#define FRAME_RATE_HZ           25
#define FRAME_PERIOD_MS         (1000 / FRAME_RATE_HZ)


// ============================================================
// SONG CONTAINER FILE
// ============================================================
//
// The laptop packs the generator's two binaries into one file so a
// song is transferred, CRC-checked and cached as a single unit:
//
//   song_header_t             12 bytes
//   frames[frame_count]        6 bytes each  (led_visualisation_data.bin)
//   notes[note_count]          7 bytes each  (led_other_notes.bin)
//
// Frame layout (6 bytes):
//   byte 0      bass volume 0-255
//   byte 1      drums: bit0 kick, bit1 hi-hat,
//               bits2-4 kick intensity, bits5-7 hi-hat intensity
//   bytes 2-3   vocal colour, uint16 RGB555
//               (bits0-4 red, bits5-9 green, bits10-14 blue)
//   byte 4      vocal volume 0-255
//   byte 5      vocal onset 0-255 (unused)
//
// Note layout (7 bytes, sorted by frame then position):
//   bytes 0-1   frame the note starts on (uint16)
//   byte 2      position 0-47 in the Other section
//   bytes 3-5   red, green, blue
//   byte 6      fade: each later frame, level *= (255 - fade) / 255
// ============================================================

#define SONG_MAGIC              "LVS2"
#define SONG_FORMAT_VERSION     1

#define SONG_FRAME_SIZE         6
#define SONG_NOTE_SIZE          7

// Notes are loaded into RAM while a song plays (~63 KB at this limit).
#define SONG_MAX_NOTES          9000


typedef struct __attribute__((packed))
{
    char     magic[4];                  // SONG_MAGIC
    uint8_t  version;                   // SONG_FORMAT_VERSION
    uint8_t  reserved;
    uint16_t note_count;
    uint32_t frame_count;

} song_header_t;

_Static_assert(sizeof(song_header_t) == 12, "song_header_t must be 12 bytes");


// True if the header is valid and describes a file of exactly `file_size` bytes.
static inline bool song_header_is_valid(const song_header_t *h, uint32_t file_size)
{
    if (h->magic[0] != SONG_MAGIC[0] || h->magic[1] != SONG_MAGIC[1] ||
        h->magic[2] != SONG_MAGIC[2] || h->magic[3] != SONG_MAGIC[3])
        return false;

    if (h->version != SONG_FORMAT_VERSION)
        return false;

    if (h->frame_count == 0 || h->note_count > SONG_MAX_NOTES)
        return false;

    uint64_t expected =
        sizeof(song_header_t) +
        (uint64_t)h->frame_count * SONG_FRAME_SIZE +
        (uint64_t)h->note_count * SONG_NOTE_SIZE;

    return expected == file_size;
}


// ------------------------------------------------------------
// Message types
// ------------------------------------------------------------

#define MSG_STATE               0x01
#define MSG_SET_OFFSET          0x02

#define MSG_FILE_START          0x10
#define MSG_FILE_CHUNK          0x11
#define MSG_FILE_END            0x12


#define STATE_FLAG_PLAYING          0x01
#define STATE_FLAG_FILE_AVAILABLE   0x02


typedef struct __attribute__((packed))
{
    uint8_t  type;                      // MSG_STATE
    char     track_id[TRACK_ID_LEN];    // all zero = nothing playing
    uint8_t  flags;                     // STATE_FLAG_*
    uint32_t position_ms;
    uint16_t seq;

} msg_state_t;

_Static_assert(sizeof(msg_state_t) == 30, "msg_state_t must be 30 bytes");


typedef struct __attribute__((packed))
{
    uint8_t  type;                      // MSG_SET_OFFSET
    int16_t  offset_ms;

} msg_set_offset_t;

_Static_assert(sizeof(msg_set_offset_t) == 3, "msg_set_offset_t must be 3 bytes");


typedef struct __attribute__((packed))
{
    uint8_t  type;                      // MSG_FILE_START
    char     track_id[TRACK_ID_LEN];
    uint32_t total_size;
    uint32_t crc32;                     // standard CRC-32 (zlib.crc32)

} msg_file_start_t;

_Static_assert(sizeof(msg_file_start_t) == 31, "msg_file_start_t must be 31 bytes");


// FILE_CHUNK: type (1) + offset uint32 (4) + data (1..FILE_CHUNK_MAX_DATA)
#define FILE_CHUNK_HEADER_LEN   5

// Largest single write the ESP accepts. With a 247-byte MTU the laptop
// sends at most 244 bytes per write (239 bytes of chunk data).
#define PROTO_MAX_WRITE_LEN     256
#define FILE_CHUNK_MAX_DATA     (PROTO_MAX_WRITE_LEN - FILE_CHUNK_HEADER_LEN)


// ------------------------------------------------------------
// STATUS (ESP -> laptop)
// ------------------------------------------------------------

typedef enum
{
    TRANSFER_IDLE       = 0,
    TRANSFER_RECEIVING  = 1,
    TRANSFER_OK         = 2,
    TRANSFER_ERROR      = 3,

} transfer_state_t;


typedef enum
{
    PROTO_ERR_NONE          = 0,
    PROTO_ERR_BAD_MESSAGE   = 1,
    PROTO_ERR_NOT_RECEIVING = 2,
    PROTO_ERR_BAD_OFFSET    = 3,
    PROTO_ERR_BAD_SIZE      = 4,
    PROTO_ERR_TOO_LARGE     = 5,
    PROTO_ERR_NO_SPACE      = 6,
    PROTO_ERR_FS_WRITE      = 7,
    PROTO_ERR_SIZE_MISMATCH = 8,
    PROTO_ERR_CRC_MISMATCH  = 9,
    PROTO_ERR_RENAME        = 10,
    PROTO_ERR_TIMEOUT       = 11,
    PROTO_ERR_ABORTED       = 12,
    PROTO_ERR_BAD_FORMAT    = 13,       // song container header invalid

} proto_error_t;


typedef struct __attribute__((packed))
{
    // Track the renderer currently has loaded and ready to play.
    // All zero when nothing is loaded.
    char     track_id[TRACK_ID_LEN];

    uint8_t  transfer_state;            // transfer_state_t
    uint8_t  error_code;                // proto_error_t (last error)
    uint32_t bytes_received;

    uint16_t ack_seq;                   // seq of the last STATE processed
    int16_t  offset_ms;                 // current latency offset (from NVS)

} status_msg_t;

_Static_assert(sizeof(status_msg_t) == 32, "status_msg_t must be 32 bytes");


// ------------------------------------------------------------
// Helpers
// ------------------------------------------------------------

static inline bool track_id_is_empty(const char *id)
{
    for (int i = 0; i < TRACK_ID_LEN; i++)
    {
        if (id[i] != 0)
            return false;
    }

    return true;
}


// Spotify IDs are base62. Checking this also guarantees the ID is
// safe to use as a filename.
static inline bool track_id_is_valid(const char *id)
{
    for (int i = 0; i < TRACK_ID_LEN; i++)
    {
        char c = id[i];

        bool ok =
            (c >= '0' && c <= '9') ||
            (c >= 'a' && c <= 'z') ||
            (c >= 'A' && c <= 'Z');

        if (!ok)
            return false;
    }

    return true;
}
