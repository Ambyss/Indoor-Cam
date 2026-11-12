#include <stdio.h>
#include <string.h>
#include "esp_log.h"
#include "mqtt_client.h"

#include "secrets.h"
#include "BLE_protocol.h"
#include "gw_mqtt.h"

static const char *TAG = "GW-MQTT";

#define T_CMD_ALL     "camera/01/cmd/#"
#define T_CMD_STREAM  "camera/01/cmd/stream"
#define T_CMD_SERVO   "camera/01/cmd/servo"
#define T_CMD_CONFIG  "camera/01/cmd/config"

#define T_ST_STATUS   "camera/01/state/status"
#define T_ST_BATTERY  "camera/01/state/battery"
#define T_ST_STREAM   "camera/01/state/stream"
#define T_ST_ONLINE   "camera/01/state/online"
#define T_ST_LINK     "camera/01/state/link"
#define T_EVT_PIR     "camera/01/evt/pir"
#define T_EVT_ERROR   "camera/01/evt/error"

static esp_mqtt_client_handle_t s_cli = NULL;
static bool s_connected = false;
static gw_mqtt_cmd_cb_t s_cb = NULL;

static void pub(const char *topic, const char *data, int qos, int retain)
{
    if (!s_connected) return;
    esp_mqtt_client_publish(s_cli, topic, data, 0, qos, retain);
}

static void on_cmd(const char *topic, int tlen,
                   const char *data, int dlen)
{
    char t[64] = { 0 }, d[128] = { 0 };
    if (tlen >= (int)sizeof(t) || dlen >= (int)sizeof(d)) return;
    memcpy(t, topic, tlen);
    memcpy(d, data, dlen);

    ESP_LOGI(TAG, "RX %s = %s", t, d);
    if (!s_cb) return;

    if (strcmp(t, T_CMD_STREAM) == 0) {
        if (strstr(d, "START"))     s_cb(CMD_START_STREAM, NULL, 0);
        else if (strstr(d, "STOP")) s_cb(CMD_STOP_STREAM,  NULL, 0);
        else if (strstr(d, "STATUS")) s_cb(CMD_GET_STATUS, NULL, 0);

    } else if (strcmp(t, T_CMD_SERVO) == 0) {
        int pan = -1, tilt = -1;
        char *p = strstr(d, "\"pan\"");
        char *q = strstr(d, "\"tilt\"");
        if (p) sscanf(p, "\"pan\"%*[: ]%d", &pan);
        if (q) sscanf(q, "\"tilt\"%*[: ]%d", &tilt);

        if (pan >= 0 && tilt >= 0) {
            cam_servo_move_t mv = { .pan = pan, .tilt = tilt };
            s_cb(CMD_SERVO_MOVE, (uint8_t *)&mv, sizeof(mv));
        } else if (strstr(d, "HOME")) {
            s_cb(CMD_SERVO_HOME, NULL, 0);
        }

    } else if (strcmp(t, T_CMD_CONFIG) == 0) {
        int res = -1, q = -1;
        char *a = strstr(d, "\"resolution\"");
        char *b = strstr(d, "\"quality\"");
        if (a) sscanf(a, "\"resolution\"%*[: ]%d", &res);
        if (b) sscanf(b, "\"quality\"%*[: ]%d", &q);
        if (res >= 0 && q >= 0) {
            cam_params_t p = { .resolution = res, .jpeg_quality = q };
            s_cb(CMD_SET_PARAMS, (uint8_t *)&p, sizeof(p));
        }
    }
}

static void mqtt_event(void *args, esp_event_base_t base,
                       int32_t id, void *data)
{
    esp_mqtt_event_handle_t e = data;

    switch ((esp_mqtt_event_id_t)id) {
    case MQTT_EVENT_CONNECTED:
        s_connected = true;
        ESP_LOGI(TAG, "Broker connected");
        esp_mqtt_client_subscribe(s_cli, T_CMD_ALL, 1);
        pub(T_ST_ONLINE, "online", 1, 1);
        break;

    case MQTT_EVENT_DISCONNECTED:
        s_connected = false;
        ESP_LOGW(TAG, "Broker lost");
        break;

    case MQTT_EVENT_DATA:
        on_cmd(e->topic, e->topic_len, e->data, e->data_len);
        break;

    case MQTT_EVENT_ERROR:
        ESP_LOGE(TAG, "MQTT error");
        break;

    default:
        break;
    }
}

esp_err_t gw_mqtt_start(gw_mqtt_cmd_cb_t cb)
{
    s_cb = cb;

    esp_mqtt_client_config_t cfg = {
        .broker.address.uri            = GW_MQTT_URI,
        .credentials.username          = GW_MQTT_USER,
        .credentials.client_id         = GW_MQTT_CLIENT_ID,
        .credentials.authentication.password = GW_MQTT_PASS,

        .session.last_will.topic   = T_ST_ONLINE,
        .session.last_will.msg     = "offline",
        .session.last_will.qos     = 1,
        .session.last_will.retain  = 1,
        .session.keepalive         = 30,

        .network.reconnect_timeout_ms = 5000,
    };

    s_cli = esp_mqtt_client_init(&cfg);
    if (!s_cli) return ESP_FAIL;

    ESP_ERROR_CHECK(esp_mqtt_client_register_event(
        s_cli, ESP_EVENT_ANY_ID, mqtt_event, NULL));
    ESP_ERROR_CHECK(esp_mqtt_client_start(s_cli));

    ESP_LOGI(TAG, "MQTT client started");
    return ESP_OK;
}

bool gw_mqtt_is_connected(void) { return s_connected; }

void gw_mqtt_pub_telemetry(const cam_telemetry_t *t)
{
    char j[224];
    snprintf(j, sizeof(j),
        "{\"battery_mv\":%u,\"solar_mv\":%u,\"state\":%u,"
        "\"pan\":%d,\"tilt\":%d,\"pir\":%u,\"rssi\":%d,\"uptime\":%lu}",
        t->battery_mv, t->solar_mv, t->state,
        t->pan_angle, t->tilt_angle, t->pir_count, t->rssi,
        (unsigned long)t->uptime_s);

    pub(T_ST_STATUS, j, 0, 1);          // retained: new clients get last state

    char b[32];
    snprintf(b, sizeof(b), "%u", t->battery_mv);
    pub(T_ST_BATTERY, b, 0, 1);
}

void gw_mqtt_pub_stream_state(bool streaming, uint8_t reason)
{
    char j[96];
    if (streaming) {
        snprintf(j, sizeof(j), "{\"state\":\"STREAMING\"}");
    } else {
        snprintf(j, sizeof(j), "{\"state\":\"IDLE\",\"reason\":%u}", reason);
    }
    pub(T_ST_STREAM, j, 1, 1);
}

void gw_mqtt_pub_event(const char *sub, const char *json)
{
    char topic[64];
    snprintf(topic, sizeof(topic), "camera/01/evt/%s", sub);
    pub(topic, json, 1, 0);             // events are not retained
}

void gw_mqtt_pub_link(bool ready, int8_t rssi)
{
    char j[64];
    snprintf(j, sizeof(j), "{\"ble\":%s,\"rssi\":%d}",
             ready ? "true" : "false", rssi);
    pub(T_ST_LINK, j, 0, 1);
}