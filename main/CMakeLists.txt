idf_component_register(SRCS "blink_example_main.c"
                       INCLUDE_DIRS "."
                       REQUIRES spiffs esp_timer)

spiffs_create_partition_image(storage ../storage FLASH_IN_PROJECT)
