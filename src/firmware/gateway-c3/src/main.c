#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "nvs_flash.h"

#include "BLE_protocol.h"
#include "gw_ble.h"
#include "gw_wifi.h"
#include "gw_mqtt.h"

static const char *TAG = "CAM-GW";

// MQTT -> BLE
static void on_mqtt_cmd(uint8_t cmd, const uint8_t *payload, size_t len)
{
    if (!gw_ble_is_ready()) {
        ESP_LOGW(TAG, "Command 0x%02X dropped: no BLE link", cmd);
        gw_mqtt_pub_event("error", "{\"code\":\"NO_BLE_LINK\"}");
        return;
    }
    int rc = gw_ble_send_cmd(cmd, payload, len);
    ESP_LOGI(TAG, "BLE cmd 0x%02X, rc=%d", cmd, rc);
}

// BLE -> MQTT
static void on_cam_event(uint8_t evt, const uint8_t *payload, size_t len)
{
    switch (evt) {
    case EVT_STATUS:
        if (len >= sizeof(cam_telemetry_t)) {
            cam_telemetry_t t;
            memcpy(&t, payload, sizeof(t));
            gw_mqtt_pub_telemetry(&t);
        }
        break;

    case EVT_STREAM_STARTED:
        gw_mqtt_pub_stream_state(true, 0);
        gw_ble_set_stream_mode(true);      // free S3 airtime for Wi-Fi
        break;

    case EVT_STREAM_STOPPED:
        gw_mqtt_pub_stream_state(false, len > 0 ? payload[0] : 255);
        gw_ble_set_stream_mode(false);
        break;

    case EVT_PIR_TRIGGERED:
        gw_mqtt_pub_event("pir", "{\"trigger\":true}");
        break;

    case EVT_LOW_BATTERY: {
        char j[48];
        uint16_t mv = (len >= 2) ? *(const uint16_t *)payload : 0;
        snprintf(j, sizeof(j), "{\"code\":\"LOW_BATTERY\",\"mv\":%u}", mv);
        gw_mqtt_pub_event("error", j);
        break;
    }

    default:
        ESP_LOGW(TAG, "Unknown event 0x%02X", evt);
    }
}

static void on_link_change(bool connected)
{
    ESP_LOGI(TAG, "BLE link: %s", connected ? "READY" : "lost");
    gw_mqtt_pub_link(connected, 0);
}

void app_main(void)
{
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        err = nvs_flash_init();
    }
    ESP_ERROR_CHECK(err);

    ESP_LOGI(TAG, "=== CamGateway start ===");

    gw_wifi_start();
    gw_mqtt_start(on_mqtt_cmd);
    gw_ble_init(on_cam_event, on_link_change);

    while (1) {
        vTaskDelay(pdMS_TO_TICKS(30000));

        if (gw_ble_is_ready()) {
            gw_ble_send_cmd(CMD_GET_STATUS, NULL, 0);
        } else {
            ESP_LOGI(TAG, "BLE: waiting (Wi-Fi=%d MQTT=%d)",
                     gw_wifi_is_connected(), gw_mqtt_is_connected());
        }
    }
}