#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_heap_caps.h"

static const char *TAG = "CATCAM";

void app_main(void)
{
    ESP_LOGI(TAG, "=== CatCam ВУ-1 (ESP32-S3) старт ===");

    // Перевірка PSRAM — головне, що треба підтвердити
    size_t psram = heap_caps_get_total_size(MALLOC_CAP_SPIRAM);
    ESP_LOGI(TAG, "PSRAM total: %u bytes (%.1f MB)",
             (unsigned)psram, psram / (1024.0 * 1024.0));

    size_t free_internal = heap_caps_get_free_size(MALLOC_CAP_INTERNAL);
    ESP_LOGI(TAG, "Internal RAM free: %u bytes", (unsigned)free_internal);

    if (psram == 0) {
        ESP_LOGE(TAG, "!!! PSRAM НЕ ЗНАЙДЕНА - перевір sdkconfig OPI !!!");
    } else {
        ESP_LOGI(TAG, "PSRAM OK");
    }

    // Живий цикл
    while (1) {
        ESP_LOGI(TAG, "alive, uptime %lld s", esp_log_timestamp() / 1000);
        vTaskDelay(pdMS_TO_TICKS(5000));
    }
}