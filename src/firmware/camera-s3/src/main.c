#include <esp_log.h>
#include "cam_ble.h"
#include "nvs_flash.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "BLE_protocol.h"
#include "cam_camera.h"
#include "cam_power.h"
#include "cam_servo.h"
#include "cam_state.h"
#include "cam_stream.h"
#include "cam_wifi.h"
#include "esp_coexist.h"

const char* TAG = "S3-CAM";

static void on_cmd(uint8_t cmd, const uint8_t* payload, size_t len)
{
    cam_state_post_cmd(cmd, payload, len);
}

void app_main(void)
{
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES)
    {
        ESP_ERROR_CHECK(nvs_flash_erase());
        err = nvs_flash_init();
    }
    ESP_ERROR_CHECK(err);

    ESP_LOGI(TAG, "=== CamNode start ===");

    ESP_LOGI(TAG, "INIT power");
    cam_power_init();
    ESP_LOGI(TAG, "INIT wifi");
    cam_wifi_init();
    ESP_LOGI(TAG, "INIT stream");
    cam_stream_init();
    ESP_LOGI(TAG, "INIT servo");
    cam_servo_init();
    esp_coex_preference_set(ESP_COEX_PREFER_BT);
    cam_ble_init(on_cmd);
    cam_state_init();
    cam_servo_power_on();
    vTaskDelay(500);
    cam_servo_move(60, 90);
    vTaskDelay(500);
    cam_servo_move(120, 90);
    cam_servo_power_off();
    while (1)
    {
        vTaskDelay(pdMS_TO_TICKS(10000));
        ESP_LOGI(TAG, "State: %d, BLE %s",
                 cam_state_get(),
                 cam_ble_is_connected() ? "connected" : "advertising");
    }
}
