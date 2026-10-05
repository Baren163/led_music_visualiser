#pragma once

#include "protocol.h"


#define BLE_DEVICE_NAME         "LED-Visualiser"
#define BLE_PREFERRED_MTU       247


// Starts NimBLE, registers the GATT service and begins advertising.
void ble_init(void);

// Sends a STATUS notification if a client is subscribed.
void ble_notify_status(const status_msg_t *status);
