#pragma once

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

typedef void (*gw_evt_handler_t)(uint8_t evt_code,
                                 const uint8_t *payload,
                                 size_t len);

typedef void (*gw_link_handler_t)(bool connected);

void gw_ble_init(gw_evt_handler_t evt_cb, gw_link_handler_t link_cb);
bool gw_ble_is_ready(void);

int  gw_ble_send_cmd(uint8_t cmd, const void *payload, size_t len);

int gw_ble_set_stream_mode(bool streaming);

int  gw_ble_get_rssi(int8_t *out_rssi);
void gw_ble_log_phy(void);
void gw_ble_log_conn_params(void);
int  gw_ble_request_slow_params(void);