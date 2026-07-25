#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_heap_caps.h"
#include "BLE_protocol.h"

static const char *TAG = "CATCAM";

void app_main(void)
{


    // Живий цикл
    while (1) {
        ESP_LOGI(TAG, "Protocol v%d, telemetry struct = %u bytes",
                 CAM_PROTOCOL_VERSION, (unsigned)sizeof(catcam_telemetry_t));
        vTaskDelay(pdMS_TO_TICKS(5000));
    }
}