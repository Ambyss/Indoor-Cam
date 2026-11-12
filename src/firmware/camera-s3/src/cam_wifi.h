#pragma once

#include <stdbool.h>
#include "esp_err.h"

esp_err_t cam_wifi_init(void);    // once at boot
esp_err_t cam_wifi_start(void);   // on WAKING
void      cam_wifi_stop(void);    // on SHUTTING_DOWN

bool      cam_wifi_is_connected(void);
int       cam_wifi_rssi(void);
int64_t   cam_wifi_connect_time_ms(void);