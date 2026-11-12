#include <string.h>
#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "driver/ledc.h"
#include "driver/gpio.h"
#include "nvs.h"

#include "BLE_protocol.h"
#include "cam_pins.h"
#include "cam_servo.h"

static const char *TAG = "CAM-SRV";

#define SRV_TIMER        LEDC_TIMER_1        // TIMER_0 is camera XCLK
#define SRV_CH_PAN       LEDC_CHANNEL_1
#define SRV_CH_TILT      LEDC_CHANNEL_2
#define SRV_MODE         LEDC_LOW_SPEED_MODE
#define SRV_RES          LEDC_TIMER_14_BIT
#define SRV_FREQ_HZ      50
#define SRV_MAX_DUTY     ((1 << 14) - 1)
#define SRV_PERIOD_US    20000

#define TICK_MS          20                  // 50 Hz
#define DT               (TICK_MS / 1000.0f)
#define V_MAX            60.0f               // deg/s
#define ACCEL            150.0f              // deg/s^2, full speed in 0.4 s
#define EPS              0.3f                // deadband, deg

#define VEL_TIMEOUT_MS   500                 // no command -> stop
#define IDLE_RELEASE_MS  4000                // idle time before PWM release

#define NVS_NS           "camsrv"

typedef struct {
    float   pos;
    float   vel;
    float   target;
    float   cmd_vel;      // deg/s
    bool    vel_mode;     // joystick mode
    int16_t deg_min, deg_max;
    uint16_t us_min, us_max;
    ledc_channel_t ch;
    bool    released;     // PWM off
    bool    hold;         // never release PWM
    int64_t still_since;  // ms
} axis_t;

static axis_t s_pan  = { .ch = SRV_CH_PAN,  .hold = false };
static axis_t s_tilt = { .ch = SRV_CH_TILT, .hold = true  };  // holds against gravity

static volatile bool    s_powered = false;
static volatile int64_t s_vel_stamp = 0;

static float clampf(float v, float lo, float hi)
{
    return v < lo ? lo : (v > hi ? hi : v);
}

static uint32_t axis_duty(const axis_t *a)
{
    float t  = (a->pos - a->deg_min) / (float)(a->deg_max - a->deg_min);
    float us = a->us_min + (a->us_max - a->us_min) * clampf(t, 0.0f, 1.0f);
    return (uint32_t)(us * SRV_MAX_DUTY / SRV_PERIOD_US);
}

static void axis_write(axis_t *a)
{
    if (!s_powered) return;

    if (a->released) {
        a->released = false;
        ESP_LOGD(TAG, "Channel %d enabled", a->ch);
    }
    ledc_set_duty(SRV_MODE, a->ch, axis_duty(a));
    ledc_update_duty(SRV_MODE, a->ch);
}

static void axis_release(axis_t *a)
{
    if (a->released || !s_powered) return;
    ledc_stop(SRV_MODE, a->ch, 0);
    a->released = true;
    ESP_LOGD(TAG, "Channel %d released", a->ch);
}

// Trapezoidal motion profile
static bool axis_step(axis_t *a)
{
    float v_target;

    if (a->vel_mode) {
        v_target = a->cmd_vel;
        // near limit: force brake
        if ((v_target > 0 && a->pos >= a->deg_max - EPS) ||
            (v_target < 0 && a->pos <= a->deg_min + EPS)) {
            v_target = 0.0f;
        }
    } else {
        float remain = a->target - a->pos;

        if (fabsf(remain) < EPS && fabsf(a->vel) < 1.0f) {
            a->pos = a->target;
            a->vel = 0.0f;
            return false;
        }

        // braking distance: v^2 / 2a
        float brake = (a->vel * a->vel) / (2.0f * ACCEL);
        if (fabsf(remain) <= brake) {
            v_target = 0.0f;
        } else {
            v_target = (remain > 0) ? V_MAX : -V_MAX;
        }
    }

    // limited accel prevents current spikes
    float dv = v_target - a->vel;
    float dv_max = ACCEL * DT;
    if (dv >  dv_max) dv =  dv_max;
    if (dv < -dv_max) dv = -dv_max;
    a->vel += dv;
    a->vel = clampf(a->vel, -V_MAX, V_MAX);

    float next = a->pos + a->vel * DT;

    if (next <= a->deg_min) { next = a->deg_min; a->vel = 0.0f; }
    if (next >= a->deg_max) { next = a->deg_max; a->vel = 0.0f; }

    bool moved = fabsf(next - a->pos) > 0.01f;
    a->pos = next;
    if (!a->vel_mode) a->target = clampf(a->target, a->deg_min, a->deg_max);
    return moved;
}

static void servo_task(void *arg)
{
    int64_t last_save = 0;

    while (1) {
        vTaskDelay(pdMS_TO_TICKS(TICK_MS));
        if (!s_powered) continue;

        int64_t now = esp_timer_get_time() / 1000;

        // Joystick watchdog: no commands -> brake
        if ((s_pan.vel_mode || s_tilt.vel_mode) &&
            now - s_vel_stamp > VEL_TIMEOUT_MS) {
            s_pan.cmd_vel = s_tilt.cmd_vel = 0.0f;
        }

        bool m1 = axis_step(&s_pan);
        bool m2 = axis_step(&s_tilt);

        if (m1) axis_write(&s_pan);
        if (m2) axis_write(&s_tilt);

        // Release idle axes, saves 10-20 mA each
        axis_t *ax[2] = { &s_pan, &s_tilt };
        for (int i = 0; i < 2; i++) {
            axis_t *a = ax[i];
            bool moving = (i == 0) ? m1 : m2;

            if (moving) {
                a->still_since = now;
            } else if (!a->hold && !a->released &&
                       now - a->still_since > IDLE_RELEASE_MS) {
                axis_release(a);
            }
        }

        // Limit NVS writes to once per 30 s
        if (!m1 && !m2 && now - last_save > 30000) {
            last_save = now;
            cam_servo_save();
        }
    }
}

static void cal_defaults(void)
{
    s_pan.us_min  = 500;  s_pan.us_max  = 2500;
    s_pan.deg_min = SERVO_PAN_MIN;  s_pan.deg_max = SERVO_PAN_MAX;

    s_tilt.us_min  = 500; s_tilt.us_max = 2500;
    s_tilt.deg_min = SERVO_TILT_MIN; s_tilt.deg_max = SERVO_TILT_MAX;
}

static void nvs_load(void)
{
    cal_defaults();

    nvs_handle_t h;
    if (nvs_open(NVS_NS, NVS_READONLY, &h) != ESP_OK) {
        ESP_LOGI(TAG, "NVS empty, using default calibration");
        s_pan.pos = s_pan.target = 90.0f;
        s_tilt.pos = s_tilt.target = 90.0f;
        return;
    }

    cam_servo_cal_t c;
    size_t sz = sizeof(c);
    if (nvs_get_blob(h, "cal", &c, &sz) == ESP_OK && sz == sizeof(c)) {
        s_pan.us_min = c.pan_us_min;   s_pan.us_max = c.pan_us_max;
        s_tilt.us_min = c.tilt_us_min; s_tilt.us_max = c.tilt_us_max;
        s_pan.deg_min = c.pan_deg_min; s_pan.deg_max = c.pan_deg_max;
        s_tilt.deg_min = c.tilt_deg_min; s_tilt.deg_max = c.tilt_deg_max;
        ESP_LOGI(TAG, "Calibration loaded");
    }

    int16_t p = 90, t = 90;
    nvs_get_i16(h, "pan",  &p);
    nvs_get_i16(h, "tilt", &t);
    nvs_close(h);

    s_pan.pos  = s_pan.target  = clampf(p, s_pan.deg_min,  s_pan.deg_max);
    s_tilt.pos = s_tilt.target = clampf(t, s_tilt.deg_min, s_tilt.deg_max);

    ESP_LOGI(TAG, "Position: pan=%.0f tilt=%.0f", s_pan.pos, s_tilt.pos);
}

void cam_servo_save(void)
{
    nvs_handle_t h;
    if (nvs_open(NVS_NS, NVS_READWRITE, &h) != ESP_OK) return;

    cam_servo_cal_t c = {
        .pan_us_min = s_pan.us_min,   .pan_us_max = s_pan.us_max,
        .tilt_us_min = s_tilt.us_min, .tilt_us_max = s_tilt.us_max,
        .pan_deg_min = s_pan.deg_min, .pan_deg_max = s_pan.deg_max,
        .tilt_deg_min = s_tilt.deg_min, .tilt_deg_max = s_tilt.deg_max,
    };
    nvs_set_blob(h, "cal", &c, sizeof(c));
    nvs_set_i16(h, "pan",  (int16_t)lroundf(s_pan.pos));
    nvs_set_i16(h, "tilt", (int16_t)lroundf(s_tilt.pos));
    nvs_commit(h);
    nvs_close(h);
}

esp_err_t cam_servo_init(void)
{
    gpio_config_t io = {
        .pin_bit_mask = 1ULL << CAM_PIN_MOSFET_SERVO,
        .mode         = GPIO_MODE_OUTPUT,
        .pull_down_en = GPIO_PULLDOWN_ENABLE,
        .pull_up_en   = GPIO_PULLUP_DISABLE,
    };
    ESP_ERROR_CHECK(gpio_config(&io));
    gpio_set_level(CAM_PIN_MOSFET_SERVO, 0);

    gpio_set_direction(CAM_PIN_SERVO_PAN,  GPIO_MODE_OUTPUT);
    gpio_set_direction(CAM_PIN_SERVO_TILT, GPIO_MODE_OUTPUT);
    gpio_set_level(CAM_PIN_SERVO_PAN,  0);
    gpio_set_level(CAM_PIN_SERVO_TILT, 0);

    nvs_load();

    xTaskCreate(servo_task, "cam_servo", 3072, NULL, 4, NULL);
    ESP_LOGI(TAG, "Initialized, power off");
    return ESP_OK;
}

esp_err_t cam_servo_power_on(void)
{
    if (s_powered) return ESP_OK;

    gpio_set_level(CAM_PIN_MOSFET_SERVO, 1);
    vTaskDelay(pdMS_TO_TICKS(50));

    ledc_timer_config_t tcfg = {
        .speed_mode      = SRV_MODE,
        .timer_num       = SRV_TIMER,
        .duty_resolution = SRV_RES,
        .freq_hz         = SRV_FREQ_HZ,
        .clk_cfg         = LEDC_USE_XTAL_CLK,   // independent of DFS
    };
    esp_err_t err = ledc_timer_config(&tcfg);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "ledc_timer_config: 0x%x", err);
        gpio_set_level(CAM_PIN_MOSFET_SERVO, 0);
        return err;
    }

    s_powered = true;

    axis_t *ax[2] = { &s_pan, &s_tilt };
    const int pins[2] = { CAM_PIN_SERVO_PAN, CAM_PIN_SERVO_TILT };

    for (int i = 0; i < 2; i++) {
        ledc_channel_config_t ch = {
            .gpio_num   = pins[i],
            .speed_mode = SRV_MODE,
            .channel    = ax[i]->ch,
            .timer_sel  = SRV_TIMER,
            .duty       = axis_duty(ax[i]),
            .hpoint     = 0,
        };
        ESP_ERROR_CHECK(ledc_channel_config(&ch));
        ax[i]->released    = false;
        ax[i]->vel         = 0.0f;
        ax[i]->vel_mode    = false;
        ax[i]->still_since = esp_timer_get_time() / 1000;
    }

    ESP_LOGI(TAG, "Power on, pan=%.0f tilt=%.0f",
             s_pan.pos, s_tilt.pos);
    return ESP_OK;
}

void cam_servo_power_off(void)
{
    if (!s_powered) return;

    cam_servo_save();
    s_powered = false;

    ledc_stop(SRV_MODE, SRV_CH_PAN,  0);
    ledc_stop(SRV_MODE, SRV_CH_TILT, 0);
    gpio_set_level(CAM_PIN_SERVO_PAN,  0);
    gpio_set_level(CAM_PIN_SERVO_TILT, 0);

    vTaskDelay(pdMS_TO_TICKS(10));
    gpio_set_level(CAM_PIN_MOSFET_SERVO, 0);

    s_pan.vel = s_tilt.vel = 0.0f;
    ESP_LOGI(TAG, "Power off");
}

bool cam_servo_is_powered(void) { return s_powered; }

void cam_servo_move(int16_t pan, int16_t tilt)
{
    s_pan.vel_mode  = false;
    s_tilt.vel_mode = false;
    s_pan.target  = clampf(pan,  s_pan.deg_min,  s_pan.deg_max);
    s_tilt.target = clampf(tilt, s_tilt.deg_min, s_tilt.deg_max);
    s_pan.still_since = s_tilt.still_since = esp_timer_get_time() / 1000;
    ESP_LOGI(TAG, "Target: pan=%.0f tilt=%.0f", s_pan.target, s_tilt.target);
}

void cam_servo_set_pan(int16_t deg)
{
    s_pan.vel_mode = false;
    s_pan.target = clampf(deg, s_pan.deg_min, s_pan.deg_max);
    s_pan.still_since = esp_timer_get_time() / 1000;
}

void cam_servo_set_tilt(int16_t deg)
{
    s_tilt.vel_mode = false;
    s_tilt.target = clampf(deg, s_tilt.deg_min, s_tilt.deg_max);
    s_tilt.still_since = esp_timer_get_time() / 1000;
}

void cam_servo_home(void)
{
    cam_servo_move((s_pan.deg_min + s_pan.deg_max) / 2,
                   (s_tilt.deg_min + s_tilt.deg_max) / 2);
}

void cam_servo_velocity(int8_t vx, int8_t vy)
{
    if (vx > 100) vx = 100;
    if (vx < -100) vx = -100;
    if (vy > 100) vy = 100;
    if (vy < -100) vy = -100;

    s_vel_stamp = esp_timer_get_time() / 1000;

    if (vx == 0 && vy == 0) {
        // stop: hold current position
        s_pan.cmd_vel = s_tilt.cmd_vel = 0.0f;
        s_pan.vel_mode = s_tilt.vel_mode = false;
        s_pan.target  = s_pan.pos;
        s_tilt.target = s_tilt.pos;
        return;
    }

    s_pan.vel_mode  = true;
    s_tilt.vel_mode = true;
    s_pan.cmd_vel  = V_MAX * vx / 100.0f;
    s_tilt.cmd_vel = V_MAX * vy / 100.0f;
    s_pan.still_since = s_tilt.still_since = s_vel_stamp;
}

int16_t cam_servo_get_pan(void)  { return (int16_t)lroundf(s_pan.pos); }
int16_t cam_servo_get_tilt(void) { return (int16_t)lroundf(s_tilt.pos); }

bool cam_servo_is_moving(void)
{
    return fabsf(s_pan.vel) > 0.1f || fabsf(s_tilt.vel) > 0.1f;
}

esp_err_t cam_servo_set_cal(const cam_servo_cal_t *c)
{
    if (c->pan_us_min >= c->pan_us_max || c->tilt_us_min >= c->tilt_us_max) {
        ESP_LOGW(TAG, "Invalid calibration rejected");
        return ESP_ERR_INVALID_ARG;
    }
    if (c->pan_us_min < 400 || c->pan_us_max > 2600 ||
        c->tilt_us_min < 400 || c->tilt_us_max > 2600) {
        ESP_LOGW(TAG, "Pulse width out of safe range");
        return ESP_ERR_INVALID_ARG;
    }

    s_pan.us_min = c->pan_us_min;    s_pan.us_max = c->pan_us_max;
    s_tilt.us_min = c->tilt_us_min;  s_tilt.us_max = c->tilt_us_max;
    s_pan.deg_min = c->pan_deg_min;  s_pan.deg_max = c->pan_deg_max;
    s_tilt.deg_min = c->tilt_deg_min; s_tilt.deg_max = c->tilt_deg_max;

    cam_servo_save();
    ESP_LOGI(TAG, "Calibration updated");
    return ESP_OK;
}

void cam_servo_get_cal(cam_servo_cal_t *o)
{
    o->pan_us_min = s_pan.us_min;    o->pan_us_max = s_pan.us_max;
    o->tilt_us_min = s_tilt.us_min;  o->tilt_us_max = s_tilt.us_max;
    o->pan_deg_min = s_pan.deg_min;  o->pan_deg_max = s_pan.deg_max;
    o->tilt_deg_min = s_tilt.deg_min; o->tilt_deg_max = s_tilt.deg_max;
}