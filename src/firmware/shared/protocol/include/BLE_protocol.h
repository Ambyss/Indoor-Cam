#pragma once

#include <stdint.h>

#define CAM_PROTOCOL_VERSION   1

// Gateway -> Cam commands
typedef enum {
    CMD_START_STREAM = 0x01,
    CMD_STOP_STREAM  = 0x02,
    CMD_SERVO_PAN    = 0x03,  // payload: int16 deg
    CMD_SERVO_TILT   = 0x04,  // payload: int16 deg
    CMD_GET_STATUS   = 0x05,
    CMD_SET_PARAMS   = 0x06,  // payload: cam_params_t
    CMD_SERVO_MOVE   = 0x07,  // payload: cam_servo_move_t
    CMD_SERVO_VEL    = 0x08,  // payload: cam_servo_vel_t (joystick)
    CMD_SERVO_CAL    = 0x09,  // payload: cam_servo_cal_t
    CMD_SERVO_HOME   = 0x0A,
} cam_cmd_t;

typedef struct __attribute__((packed)) {
    int16_t pan;
    int16_t tilt;
} cam_servo_move_t;

// Percent of max speed, -100..100
typedef struct __attribute__((packed)) {
    int8_t vx;    // + right
    int8_t vy;    // + up
} cam_servo_vel_t;

// Per-unit calibration, MG90S units vary
typedef struct __attribute__((packed)) {
    uint16_t pan_us_min,  pan_us_max;
    uint16_t tilt_us_min, tilt_us_max;
    int16_t  pan_deg_min, pan_deg_max;
    int16_t  tilt_deg_min, tilt_deg_max;
} cam_servo_cal_t;

// Cam -> Gateway events
typedef enum {
    EVT_STATUS         = 0x80,  // payload: cam_telemetry_t
    EVT_PIR_TRIGGERED  = 0x81,  // payload: uint32 timestamp_ms
    EVT_STREAM_STARTED = 0x82,
    EVT_STREAM_STOPPED = 0x83,  // payload: uint8 cam_stop_reason_t
    EVT_LOW_BATTERY    = 0x84,  // payload: uint16 battery_mv
} cam_evt_t;

typedef enum {
    STATE_SLEEPING      = 0,
    STATE_WAKING        = 1,
    STATE_STREAMING     = 2,
    STATE_SHUTTING_DOWN = 3,
    STATE_FAULT         = 4,
} cam_state_t;

// EVT_STREAM_STOPPED payload
typedef enum {
    STOP_BY_COMMAND     = 0,
    STOP_SESSION_LIMIT  = 1,
    STOP_HEARTBEAT_LOST = 2,
    STOP_LOW_BATTERY    = 3,
} cam_stop_reason_t;

typedef enum {
    RES_QVGA = 0,   // 320x240
    RES_VGA  = 1,   // 640x480
    RES_SVGA = 2,   // 800x600
    RES_XGA  = 3,   // 1024x768
} cam_resolution_t;

typedef struct __attribute__((packed)) {
    uint8_t resolution;   // cam_resolution_t
    uint8_t jpeg_quality; // 10..30, lower = better
} cam_params_t;

// EVT_STATUS payload, reply to CMD_GET_STATUS
typedef struct __attribute__((packed)) {
    uint16_t battery_mv;
    uint16_t solar_mv;
    uint8_t  state;       // cam_state_t
    int16_t  pan_angle;   // deg
    int16_t  tilt_angle;  // deg
    uint16_t pir_count;
    int8_t   rssi;        // BLE link, dBm
    uint32_t uptime_s;
} cam_telemetry_t;

// Servo limits, deg
#define SERVO_PAN_MIN    0
#define SERVO_PAN_MAX    180
#define SERVO_TILT_MIN   30
#define SERVO_TILT_MAX   150

#define CAM_CMD_HEADER_SIZE   1
#define CAM_MAX_PAYLOAD_SIZE   32
#define CAM_MAX_PACKET_SIZE  (CAM_CMD_HEADER_SIZE + CAM_MAX_PAYLOAD_SIZE)

// GATT: custom 128-bit UUIDs (base A5C1xxxx-...)
#define CAM_SVC_UUID128 \
BLE_UUID128_INIT(0x9e, 0x5d, 0x1e, 0x47, 0x5c, 0x13, 0x43, 0xa0, \
0xb4, 0x77, 0x3b, 0x0f, 0x01, 0x00, 0xc1, 0xa5)

// Commands: C3 -> S3, WRITE
#define CAM_CHR_CMD_UUID128 \
BLE_UUID128_INIT(0x9e, 0x5d, 0x1e, 0x47, 0x5c, 0x13, 0x43, 0xa0, \
0xb4, 0x77, 0x3b, 0x0f, 0x02, 0x00, 0xc1, 0xa5)

// Events/telemetry: S3 -> C3, NOTIFY
#define CAM_CHR_EVT_UUID128 \
BLE_UUID128_INIT(0x9e, 0x5d, 0x1e, 0x47, 0x5c, 0x13, 0x43, 0xa0, \
0xb4, 0x77, 0x3b, 0x0f, 0x03, 0x00, 0xc1, 0xa5)

#define CAM_DEVICE_NAME   "CatCam-01"

// Conn params, NimBLE units: interval 1.25 ms, timeout 10 ms
// Rule: (1 + latency) * itvl_max * 2 < timeout

// Idle: (1 + 4) * 500 ms * 2 = 5 s < 16 s
#define CAM_CONN_ITVL_MIN        400   // 500 ms
#define CAM_CONN_ITVL_MAX        400   // 500 ms
#define CAM_CONN_LATENCY           4
#define CAM_CONN_TIMEOUT        1600   // 16 s

// Streaming, BLE yields air to Wi-Fi: (1 + 0) * 4 s * 2 = 8 s < 32 s
#define CAM_CONN_ITVL_STREAM    3200   // 4 s
#define CAM_CONN_LATENCY_STREAM    0
#define CAM_CONN_TIMEOUT_STREAM 3200   // 32 s