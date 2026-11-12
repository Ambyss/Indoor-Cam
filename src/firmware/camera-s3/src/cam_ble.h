#pragma once

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

typedef void (*cam_cmd_handler_t)(uint8_t cmd,
                                  const uint8_t *payload,
                                  size_t len);

void cam_ble_init(cam_cmd_handler_t handler);
bool cam_ble_is_connected(void);
int  cam_ble_send_event(uint8_t evt_code, const void *payload, size_t len);