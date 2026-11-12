#include "esp_log.h"
#include "esp_pm.h"

#include "cam_power.h"

static const char *TAG = "CAM-PWR";

static esp_pm_lock_handle_t s_lock;
static bool s_held = false;

void cam_power_init(void)
{
    esp_pm_config_t cfg = {
        .max_freq_mhz       = 240,
        .min_freq_mhz       = 40,    // S3 XTAL freq
        .light_sleep_enable = true,
    };
    ESP_ERROR_CHECK(esp_pm_configure(&cfg));

    ESP_ERROR_CHECK(esp_pm_lock_create(ESP_PM_APB_FREQ_MAX, 0,
                                       "cam_active", &s_lock));

    ESP_LOGI(TAG, "PM: DFS 40-240 MHz, light sleep enabled");
}

void cam_power_hold(void)
{
    if (!s_held) {
        esp_pm_lock_acquire(s_lock);
        s_held = true;
        ESP_LOGI(TAG, "Sleep locked");
    }
}

void cam_power_release(void)
{
    if (s_held) {
        esp_pm_lock_release(s_lock);
        s_held = false;
        ESP_LOGI(TAG, "Sleep allowed");
    }
}

uint16_t cam_power_battery_mv(void)
{
    // TODO: real ADC read
    return 3900;
}