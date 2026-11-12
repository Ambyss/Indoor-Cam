#pragma once
#include <stdbool.h>
#include <stdint.h>
#include <stddef.h>
#include "esp_err.h"
#include "BLE_protocol.h"

typedef void (*gw_mqtt_cmd_cb_t)(uint8_t cmd,
                                 const uint8_t* payload, size_t len);

esp_err_t gw_mqtt_start(gw_mqtt_cmd_cb_t cb);
bool gw_mqtt_is_connected(void);

void gw_mqtt_pub_telemetry(const cam_telemetry_t* t);
void gw_mqtt_pub_stream_state(bool streaming, uint8_t reason);
void gw_mqtt_pub_event(const char* sub, const char* json);
void gw_mqtt_pub_link(bool ble_ready, int8_t rssi);
