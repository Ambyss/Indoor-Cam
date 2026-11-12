#include <string.h>
#include "esp_log.h"
#include "esp_timer.h"
#include "driver/gpio.h"
#include "esp_camera.h"

#include "cam_pins.h"
#include "cam_camera.h"

static const char *TAG = "CAM-CAM";

static bool s_active = false;

static camera_config_t s_cfg = {
    .pin_pwdn     = CAM_PIN_PWDN,
    .pin_reset    = CAM_PIN_RESET,
    .pin_xclk     = CAM_PIN_XCLK,
    .pin_sccb_sda = CAM_PIN_SIOD,
    .pin_sccb_scl = CAM_PIN_SIOC,

    .pin_d7 = CAM_PIN_D7,
    .pin_d6 = CAM_PIN_D6,
    .pin_d5 = CAM_PIN_D5,
    .pin_d4 = CAM_PIN_D4,
    .pin_d3 = CAM_PIN_D3,
    .pin_d2 = CAM_PIN_D2,
    .pin_d1 = CAM_PIN_D1,
    .pin_d0 = CAM_PIN_D0,

    .pin_vsync = CAM_PIN_VSYNC,
    .pin_href  = CAM_PIN_HREF,
    .pin_pclk  = CAM_PIN_PCLK,

    .xclk_freq_hz = 10000000, // not 10000000 because cam tire mounted
    .ledc_timer   = LEDC_TIMER_0,     // servos use TIMER_1
    .ledc_channel = LEDC_CHANNEL_0,

    .pixel_format = PIXFORMAT_JPEG,   // hardware JPEG
    .frame_size   = FRAMESIZE_SVGA,
    .jpeg_quality = 12,
    .fb_count     = 2,
    .fb_location  = CAMERA_FB_IN_PSRAM,
    .grab_mode    = CAMERA_GRAB_LATEST,
};

static framesize_t map_resolution(cam_resolution_t r)
{
    switch (r) {
    case RES_QVGA: return FRAMESIZE_QVGA;
    case RES_VGA:  return FRAMESIZE_VGA;
    case RES_SVGA: return FRAMESIZE_SVGA;
    case RES_XGA:  return FRAMESIZE_XGA;
    default:       return FRAMESIZE_SVGA;
    }
}

esp_err_t cam_camera_start(void)
{
    if (s_active) return ESP_OK;

    int64_t t0 = esp_timer_get_time();

    esp_err_t err = esp_camera_init(&s_cfg);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "esp_camera_init failed 0x%x", err);
        return err;
    }

    sensor_t *s = esp_camera_sensor_get();
    if (s == NULL) {
        ESP_LOGE(TAG, "Sensor not available");
        esp_camera_deinit();
        return ESP_FAIL;
    }

    ESP_LOGI(TAG, "Sensor PID=0x%04X", s->id.PID);
    if (s->id.PID == OV3660_PID) {
        ESP_LOGI(TAG, "OV3660 detected");
        s->set_vflip(s, 1);
        s->set_brightness(s, 1);
        s->set_saturation(s, -2);
    } else {
        ESP_LOGW(TAG, "Expected OV3660, got PID 0x%04X", s->id.PID);
    }

    s_active = true;
    ESP_LOGI(TAG, "Camera started in %lld ms",
             (esp_timer_get_time() - t0) / 1000);
    return ESP_OK;
}

void cam_camera_stop(void)
{
    if (!s_active) return;

    esp_camera_deinit();
    s_active = false;

    // PWDN high: sensor deep sleep (~1 mA -> uA)
    if (CAM_PIN_PWDN >= 0) {
        gpio_set_direction(CAM_PIN_PWDN, GPIO_MODE_OUTPUT);
        gpio_set_level(CAM_PIN_PWDN, 1);
    }

    ESP_LOGI(TAG, "Camera stopped, PWDN active");
}

bool cam_camera_is_active(void)
{
    return s_active;
}

esp_err_t cam_camera_apply_params(cam_resolution_t res, uint8_t quality)
{
    if (quality < 10) quality = 10;
    if (quality > 30) quality = 30;

    s_cfg.frame_size   = map_resolution(res);
    s_cfg.jpeg_quality = quality;

    if (!s_active) return ESP_OK;   // applied on next start

    sensor_t *s = esp_camera_sensor_get();
    if (s == NULL) return ESP_FAIL;

    s->set_framesize(s, s_cfg.frame_size);
    s->set_quality(s, quality);
    ESP_LOGI(TAG, "Params: res=%d, quality=%u", res, quality);
    return ESP_OK;
}

esp_err_t cam_camera_test_shot(void)
{
    if (!s_active) {
        ESP_LOGW(TAG, "Camera not active");
        return ESP_ERR_INVALID_STATE;
    }

    camera_fb_t *fb = esp_camera_fb_get();
    if (fb == NULL) {
        ESP_LOGE(TAG, "No frame");
        return ESP_FAIL;
    }

    ESP_LOGI(TAG, "Frame: %ux%u, %u bytes, format=%d",
             fb->width, fb->height, (unsigned)fb->len, fb->format);

    // Expect JPEG SOI: FF D8
    if (fb->len > 2) {
        ESP_LOGI(TAG, "Header: %02X %02X %s",
                 fb->buf[0], fb->buf[1],
                 (fb->buf[0] == 0xFF && fb->buf[1] == 0xD8) ? "(JPEG OK)" : "(?)");
    }

    esp_camera_fb_return(fb);
    return ESP_OK;
}

void cam_camera_benchmark(int frames)
{
    if (!s_active) {
        ESP_LOGW(TAG, "Camera not active");
        return;
    }

    size_t total = 0;
    int    ok    = 0;
    int64_t t0   = esp_timer_get_time();

    for (int i = 0; i < frames; i++) {
        camera_fb_t *fb = esp_camera_fb_get();
        if (fb == NULL) continue;
        total += fb->len;
        ok++;
        esp_camera_fb_return(fb);
    }

    int64_t dt_ms = (esp_timer_get_time() - t0) / 1000;
    if (dt_ms == 0 || ok == 0) return;

    float fps      = ok * 1000.0f / dt_ms;
    float avg_kb   = total / (float)ok / 1024.0f;
    float mbit_s   = total * 8.0f / dt_ms / 1000.0f;

    ESP_LOGI(TAG, "FPS=%.1f | frame=%.1f KB | rate=%.2f Mbit/s | %d/%d frames",
             fps, avg_kb, mbit_s, ok, frames);
}