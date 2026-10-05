#include "esp_log.h"
#include "esp_err.h"
#include "nvs_flash.h"

#include "renderer.h"
#include "storage.h"
#include "control.h"
#include "ble.h"


// ============================================================
// TASK OVERVIEW
// ============================================================
//
//  NimBLE host task  parses writes, queues them for the control task
//  control task      owns playback state, does all flash writes
//  render task       plays frames on the strip (highest app priority)
//
// See protocol.h for the BLE message formats.
// ============================================================


static const char *TAG = "audio_visualiser";


static void init_nvs(void)
{
    esp_err_t err = nvs_flash_init();

    if (err == ESP_ERR_NVS_NO_FREE_PAGES ||
        err == ESP_ERR_NVS_NEW_VERSION_FOUND)
    {
        ESP_ERROR_CHECK(nvs_flash_erase());
        err = nvs_flash_init();
    }

    ESP_ERROR_CHECK(err);
}


void app_main(void)
{
    ESP_LOGI(TAG, "Starting Audio Visualiser");

    init_nvs();

    if (storage_init() != ESP_OK)
        ESP_LOGE(TAG, "Storage unavailable, songs cannot be cached");

    renderer_init();
    control_init();
    ble_init();
}
