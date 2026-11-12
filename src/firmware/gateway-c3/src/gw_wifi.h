#pragma once
#include <stdbool.h>
#include "esp_err.h"

esp_err_t gw_wifi_start(void);
bool      gw_wifi_is_connected(void);