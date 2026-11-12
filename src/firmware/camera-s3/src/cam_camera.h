#pragma once

#include <stdbool.h>
#include <stdint.h>
#include "esp_err.h"
#include "BLE_protocol.h"

esp_err_t cam_camera_start(void);      // init + wake
void      cam_camera_stop(void);       // deinit + PWDN
bool      cam_camera_is_active(void);

esp_err_t cam_camera_apply_params(cam_resolution_t res, uint8_t quality);

// Grabs one frame, logs its size
esp_err_t cam_camera_test_shot(void);

// Local FPS benchmark, no network
void      cam_camera_benchmark(int frames);