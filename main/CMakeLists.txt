idf_component_register(SRCS "main.c"
                            "renderer.c"
                            "storage.c"
                            "control.c"
                            "ble.c"
                       INCLUDE_DIRS "."
                       REQUIRES bt nvs_flash esp_timer vfs)

# Songs are no longer flashed with the firmware: the laptop sends them
# over BLE and they are cached on the LittleFS "storage" partition.
