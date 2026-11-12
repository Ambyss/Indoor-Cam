#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "esp_task_wdt.h"

#include "BLE_protocol.h"
#include "cam_ble.h"
#include "cam_power.h"
#include "cam_state.h"

#include <esp_coexist.h>

#include "cam_camera.h"
#include "cam_servo.h"
#include "cam_wifi.h"
#include "cam_stream.h"

static const char *TAG = "CAM-STATE";

// Safety limits
#define SESSION_LIMIT_MS      (10 * 60 * 1000)
#define WAKE_TIMEOUT_MS       (30 * 1000)        // Wi-Fi connect timeout
#define FAULT_RECOVERY_MS     (60 * 1000)        // FAULT cooldown
#define BATTERY_MIN_MV         3400              // min to start stream
#define BATTERY_CRIT_MV        3200              // stops stream

#define QUEUE_LEN               8
#define TICK_MS               500

typedef struct {
    uint8_t cmd;
    uint8_t payload[CAM_MAX_PAYLOAD_SIZE];
    uint8_t len;
} state_msg_t;

static QueueHandle_t s_queue;
static cam_state_t   s_state = STATE_SLEEPING;
static int64_t       s_enter_us;      // state entry time, us
static bool s_shot_ok = false;   // test shot passed

static const char *state_name(cam_state_t s)
{
    switch (s) {
    case STATE_SLEEPING:      return "SLEEPING";
    case STATE_WAKING:        return "WAKING";
    case STATE_STREAMING:     return "STREAMING";
    case STATE_SHUTTING_DOWN: return "SHUTTING_DOWN";
    case STATE_FAULT:         return "FAULT";
    default:                  return "?";
    }
}

static int64_t elapsed_ms(void)
{
    return (esp_timer_get_time() - s_enter_us) / 1000;
}

static void transit(cam_state_t next)
{
    if (next == s_state) return;

    ESP_LOGI(TAG, "%s -> %s", state_name(s_state), state_name(next));
    s_state    = next;
    s_enter_us = esp_timer_get_time();

    switch (next) {
    case STATE_WAKING:
    cam_power_hold();
    s_shot_ok = false;
    if (cam_camera_start() != ESP_OK) {
        transit(STATE_FAULT);
        return;
    }
    cam_wifi_start();
    break;

    case STATE_STREAMING:
        esp_coex_preference_set(ESP_COEX_PREFER_WIFI);   // prefer Wi-Fi for video
        cam_servo_power_on();
        cam_stream_start();
        cam_ble_send_event(EVT_STREAM_STARTED, NULL, 0);
        break;

case STATE_SHUTTING_DOWN:
    cam_stream_stop();
    cam_servo_power_off();
    cam_camera_stop();
    cam_wifi_stop();
    break;

    case STATE_SLEEPING:
        esp_coex_preference_set(ESP_COEX_PREFER_BT);     // back to BLE
        cam_power_release();
        break;

    case STATE_FAULT:
        cam_power_release();
        break;

    default:
        break;
    }
}

static void stop_stream(cam_stop_reason_t reason)
{
    uint8_t r = (uint8_t)reason;
    cam_ble_send_event(EVT_STREAM_STOPPED, &r, sizeof(r));
    transit(STATE_SHUTTING_DOWN);
}

static void handle_cmd(const state_msg_t *m)
{
    switch (m->cmd) {

    case CMD_START_STREAM:
        if (s_state != STATE_SLEEPING) {
            ESP_LOGW(TAG, "START ignored, state %s", state_name(s_state));
            return;
        }
        if (cam_power_battery_mv() < BATTERY_MIN_MV) {
            ESP_LOGW(TAG, "START rejected: low battery");
            uint16_t mv = cam_power_battery_mv();
            cam_ble_send_event(EVT_LOW_BATTERY, &mv, sizeof(mv));
            return;
        }
        transit(STATE_WAKING);
        break;

    case CMD_STOP_STREAM:
        if (s_state == STATE_STREAMING || s_state == STATE_WAKING) {
            stop_stream(STOP_BY_COMMAND);
        }
        break;

    case CMD_SERVO_PAN:
    case CMD_SERVO_TILT:
    case CMD_SERVO_MOVE:
    case CMD_SERVO_VEL:
    case CMD_SERVO_HOME: {
            if (s_state != STATE_STREAMING) {
                ESP_LOGW(TAG, "Servo ignored while not streaming");
                return;
            }
            switch (m->cmd) {
            case CMD_SERVO_PAN:
                if (m->len >= 2) { int16_t d; memcpy(&d, m->payload, 2);
                    cam_servo_set_pan(d); }
                break;
            case CMD_SERVO_TILT:
                if (m->len >= 2) { int16_t d; memcpy(&d, m->payload, 2);
                    cam_servo_set_tilt(d); }
                break;
            case CMD_SERVO_MOVE:
                if (m->len >= sizeof(cam_servo_move_t)) {
                    cam_servo_move_t mv; memcpy(&mv, m->payload, sizeof(mv));
                    cam_servo_move(mv.pan, mv.tilt);
                }
                break;
            case CMD_SERVO_VEL:
                if (m->len >= sizeof(cam_servo_vel_t)) {
                    cam_servo_vel_t v; memcpy(&v, m->payload, sizeof(v));
                    cam_servo_velocity(v.vx, v.vy);
                }
                break;
            case CMD_SERVO_HOME:
                cam_servo_home();
                break;
            }
            break;
    }

    case CMD_SERVO_CAL:
        if (m->len >= sizeof(cam_servo_cal_t)) {
            cam_servo_cal_t c;
            memcpy(&c, m->payload, sizeof(c));
            cam_servo_set_cal(&c);
        }
        break;

    case CMD_GET_STATUS: {
        cam_telemetry_t t = {
            .battery_mv = cam_power_battery_mv(),
            .solar_mv   = 0,
            .state      = (uint8_t)s_state,
            .pan_angle  = cam_servo_get_pan(),
            .tilt_angle = cam_servo_get_tilt(),
            .pir_count  = 0,
            .rssi       = 0,
            .uptime_s   = (uint32_t)(esp_timer_get_time() / 1000000),
        };
        cam_ble_send_event(EVT_STATUS, &t, sizeof(t));
        cam_stream_send_telemetry(&t, sizeof(t));
        break;
    }

    case CMD_SET_PARAMS: {
            if (m->len < sizeof(cam_params_t)) {
                ESP_LOGW(TAG, "SET_PARAMS payload too short");
                return;
            }
            cam_params_t p;
            memcpy(&p, m->payload, sizeof(p));
            cam_camera_apply_params((cam_resolution_t)p.resolution, p.jpeg_quality);
            break;
    }

    default:
        ESP_LOGW(TAG, "Unknown command 0x%02X", m->cmd);
    }
}

static void check_timeouts(void)
{
    switch (s_state) {

    case STATE_WAKING:
    if (!s_shot_ok && elapsed_ms() > 500) {
        s_shot_ok = (cam_camera_test_shot() == ESP_OK);
        if (!s_shot_ok) {
            ESP_LOGE(TAG, "Camera returned no frame");
            transit(STATE_FAULT);
            return;
        }
    }
    if (s_shot_ok && cam_wifi_is_connected()) {
        ESP_LOGI(TAG, "Wi-Fi ready in %lld ms", cam_wifi_connect_time_ms());
        transit(STATE_STREAMING);
        return;
    }
    if (elapsed_ms() > WAKE_TIMEOUT_MS) {
        ESP_LOGE(TAG, "Wake timeout");
        transit(STATE_FAULT);
    }
    break;

case STATE_STREAMING: {
    int64_t hb = cam_stream_since_hb_ms();
    if (elapsed_ms() > SESSION_LIMIT_MS) {
        stop_stream(STOP_SESSION_LIMIT);
    } else if (cam_power_battery_mv() < BATTERY_CRIT_MV) {
        stop_stream(STOP_LOW_BATTERY);
    } else if (hb > 45000) {          // 3 missed 15 s heartbeats
        ESP_LOGW(TAG, "Heartbeat lost");
        stop_stream(STOP_HEARTBEAT_LOST);
    }
    break;
}

    case STATE_SHUTTING_DOWN:
        if (elapsed_ms() > 500) {
            transit(STATE_SLEEPING);
        }
        break;

    case STATE_FAULT:
        if (elapsed_ms() > FAULT_RECOVERY_MS) {
            transit(STATE_SLEEPING);
        }
        break;

    default:
        break;
    }
}

static void state_task(void *arg)
{
    ESP_ERROR_CHECK(esp_task_wdt_add(NULL));

    state_msg_t msg;
    s_enter_us = esp_timer_get_time();

    while (1) {
        if (xQueueReceive(s_queue, &msg, pdMS_TO_TICKS(TICK_MS)) == pdTRUE) {
            handle_cmd(&msg);
        }
        check_timeouts();
        esp_task_wdt_reset();
    }
}

void cam_state_init(void)
{
    s_queue = xQueueCreate(QUEUE_LEN, sizeof(state_msg_t));
    configASSERT(s_queue);

    cam_stream_set_cmd_cb(cam_state_post_cmd);

    xTaskCreate(state_task, "cam_state", 4096, NULL, 5, NULL);
    ESP_LOGI(TAG, "State machine started, state %s", state_name(s_state));
}

void cam_state_post_cmd(uint8_t cmd, const uint8_t *payload, size_t len)
{
    state_msg_t m = { .cmd = cmd, .len = (uint8_t)len };
    if (payload && len && len <= sizeof(m.payload)) {
        memcpy(m.payload, payload, len);
    }
    xQueueSend(s_queue, &m, 0);
}

cam_state_t cam_state_get(void)
{
    return s_state;
}