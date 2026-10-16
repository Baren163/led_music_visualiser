#pragma once

#include <stdint.h>
#include <stdbool.h>

#include "protocol.h"


typedef enum
{
    RENDERER_OFF,           // strip dark, no file
    RENDERER_IDLE,          // idle animation, no file
    RENDERER_TRANSFER,      // file transfer progress bar, no file
    RENDERER_PLAYING,       // file open, frames advancing
    RENDERER_PAUSED,        // file open, last frame frozen on the strip
    RENDERER_ENDED,         // file open, position past the last frame

} renderer_mode_t;


// Configures the LED strip and starts the render task.
void renderer_init(void);


// All commands below block until the render task has processed them.
// They must only be called from the control task.

bool renderer_open(const char *track_id, uint32_t start_frame, bool playing);
void renderer_seek(uint32_t frame);

// Draws `frame` once and freezes it (seeking while paused).
void renderer_show_still(uint32_t frame);

void renderer_pause(void);
void renderer_resume(uint32_t frame);
void renderer_idle(void);
void renderer_off(void);
void renderer_show_transfer(void);


// Non-blocking; only affects the transfer animation.
void renderer_set_progress(uint8_t percent);


renderer_mode_t renderer_get_mode(void);

// Next frame index the renderer will display.
uint32_t renderer_get_frame(void);

// Track ID of the open file, all zero when none.
void renderer_get_loaded_track(char out[TRACK_ID_LEN]);
