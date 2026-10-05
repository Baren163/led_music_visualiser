#pragma once

#include <stdint.h>

#include "protocol.h"


typedef enum
{
    CTRL_SRC_STATE_CHR,         // write to the STATE characteristic
    CTRL_SRC_FILE_CHR,          // write to the FILE characteristic
    CTRL_EVT_CONNECTED,
    CTRL_EVT_DISCONNECTED,

} ctrl_kind_t;


typedef enum
{
    CTRL_POST_OK,
    CTRL_POST_INVALID,          // malformed message
    CTRL_POST_BUSY,             // queue full

} ctrl_post_result_t;


void control_init(void);


// Called from the NimBLE host task. Validates and queues a write;
// no flash work happens here.
ctrl_post_result_t control_post_write(
    ctrl_kind_t kind,
    const uint8_t *data,
    uint16_t len
);

void control_post_event(ctrl_kind_t kind);


// Thread-safe snapshot of the current STATUS value.
void control_get_status(status_msg_t *out);
