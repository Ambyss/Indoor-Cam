#pragma once

#include <stdbool.h>
#include <stdint.h>
#include <stddef.h>
#include "esp_err.h"

// Command from bridge over TCP
typedef void (*cam_stream_cmd_cb_t)(uint8_t cmd,
                                    const uint8_t *payload,
                                    size_t len);

esp_err_t cam_stream_init(void);
void      cam_stream_start(void);
void      cam_stream_stop(void);

bool      cam_stream_is_connected(void);
int64_t   cam_stream_since_hb_ms(void);
void      cam_stream_log_stats(void);

// Test control channel (WebRTC data channel prototype)
void      cam_stream_set_cmd_cb(cam_stream_cmd_cb_t cb);
void      cam_stream_send_telemetry(const void *t, size_t len);