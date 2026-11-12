#pragma once

#include <stdint.h>
#include <stddef.h>
#include "BLE_protocol.h"

void        cam_state_init(void);
void        cam_state_post_cmd(uint8_t cmd, const uint8_t *payload, size_t len);
cam_state_t cam_state_get(void);