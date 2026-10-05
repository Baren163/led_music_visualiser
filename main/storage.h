#pragma once

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

#include "esp_err.h"

#include "protocol.h"


// Mounts LittleFS at /storage and rebuilds the song cache index.
esp_err_t storage_init(void);


void storage_track_path(const char *track_id, char *out, size_t out_len);

bool storage_has_track(const char *track_id);

// Marks a cached track as most recently used.
void storage_touch(const char *track_id);


// ------------------------------------------------------------
// File transfer. Every function returns a proto_error_t.
// On any error the partial file is deleted.
// ------------------------------------------------------------

uint8_t  storage_xfer_begin(const char *track_id, uint32_t total_size, uint32_t crc32);
uint8_t  storage_xfer_chunk(uint32_t offset, const uint8_t *data, size_t len);
uint8_t  storage_xfer_end(void);
void     storage_xfer_abort(void);

bool     storage_xfer_active(void);
void     storage_xfer_track(char out[TRACK_ID_LEN]);
uint32_t storage_xfer_received(void);
uint32_t storage_xfer_total(void);

// Microseconds since the last FILE_START/FILE_CHUNK.
int64_t  storage_xfer_idle_us(void);
