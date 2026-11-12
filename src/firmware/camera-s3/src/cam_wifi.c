#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"
#include "esp_log.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_netif.h"
#include "esp_timer.h"

#include "secrets.h"
#include "cam_wifi.h"

static const char *TAG = "CAM-WIFI";

#define WIFI_BIT_GOT_IP   BIT0
#define MAX_RETRY         10

static EventGroupHandle_t s_events;
static esp_netif_t       *s_netif    = NULL;
static bool               s_started  = false;
static int                s_retry    = 0;
static int64_t            s_t_start  = 0;
static int64_t            s_t_ready  = 0;

static void wifi_event_handler(void *arg, esp_event_base_t base,
                               int32_t id, void *data)
{
    if (base == WIFI_EVENT && id == WIFI_EVENT_STA_START) {
        esp_wifi_connect();

    } else if (base == WIFI_EVENT && id == WIFI_EVENT_STA_DISCONNECTED) {
        xEventGroupClearBits(s_events, WIFI_BIT_GOT_IP);
        if (s_started && s_retry < MAX_RETRY) {
            s_retry++;
            ESP_LOGW(TAG, "Reconnecting %d/%d", s_retry, MAX_RETRY);
            esp_wifi_connect();
        }

    } else if (base == IP_EVENT && id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t *e = (ip_event_got_ip_t *)data;
        s_retry   = 0;
        s_t_ready = esp_timer_get_time();
        ESP_LOGI(TAG, "IP " IPSTR " in %lld ms",
                 IP2STR(&e->ip_info.ip), (s_t_ready - s_t_start) / 1000);
        xEventGroupSetBits(s_events, WIFI_BIT_GOT_IP);
    }
}

esp_err_t cam_wifi_init(void)
{
    s_events = xEventGroupCreate();

    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    s_netif = esp_netif_create_default_wifi_sta();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    ESP_ERROR_CHECK(esp_event_handler_instance_register(
        WIFI_EVENT, ESP_EVENT_ANY_ID, wifi_event_handler, NULL, NULL));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(
        IP_EVENT, IP_EVENT_STA_GOT_IP, wifi_event_handler, NULL, NULL));

    wifi_config_t wc = { 0 };
    strncpy((char *)wc.sta.ssid,     CAM_WIFI_SSID, sizeof(wc.sta.ssid) - 1);
    strncpy((char *)wc.sta.password, CAM_WIFI_PASS, sizeof(wc.sta.password) - 1);
    wc.sta.threshold.authmode = WIFI_AUTH_WPA2_PSK;

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wc));
    ESP_ERROR_CHECK(esp_wifi_set_storage(WIFI_STORAGE_RAM));

    ESP_LOGI(TAG, "Wi-Fi initialized (radio off)");
    return ESP_OK;
}

esp_err_t cam_wifi_start(void)
{
    if (s_started) return ESP_OK;

    s_retry   = 0;
    s_t_start = esp_timer_get_time();
    s_t_ready = 0;
    xEventGroupClearBits(s_events, WIFI_BIT_GOT_IP);

    esp_err_t err = esp_wifi_start();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "esp_wifi_start: 0x%x", err);
        return err;
    }

    // No power save: streaming needs full bandwidth
    esp_wifi_set_ps(WIFI_PS_NONE);

    s_started = true;
    ESP_LOGI(TAG, "Wi-Fi started");
    return ESP_OK;
}

void cam_wifi_stop(void)
{
    if (!s_started) return;

    s_started = false;
    esp_wifi_disconnect();
    esp_wifi_stop();
    xEventGroupClearBits(s_events, WIFI_BIT_GOT_IP);

    ESP_LOGI(TAG, "Wi-Fi stopped");
}

bool cam_wifi_is_connected(void)
{
    if (!s_started) return false;
    return (xEventGroupGetBits(s_events) & WIFI_BIT_GOT_IP) != 0;
}

int cam_wifi_rssi(void)
{
    wifi_ap_record_t ap;
    if (esp_wifi_sta_get_ap_info(&ap) != ESP_OK) return 0;
    return ap.rssi;
}

int64_t cam_wifi_connect_time_ms(void)
{
    if (s_t_ready == 0) return -1;
    return (s_t_ready - s_t_start) / 1000;
}