#pragma once

#include <stdint.h>

#define CAM_PROTOCOL_VERSION   1

// Gateway -> Cam commands
typedef enum {
    CMD_START_STREAM = 0x01,
    CMD_STOP_STREAM  = 0x02,
    CMD_SERVO_PAN    = 0x03,  /* payload: int16 angle (deg)         */
    CMD_SERVO_TILT   = 0x04,  /* payload: int16 angle (deg)         */
    CMD_GET_STATUS   = 0x05,
    CMD_SET_PARAMS   = 0x06,  /* payload: catcam_params_t           */
} cam_cmd_t;

// Cam -> Gateway commands
typedef enum {
    EVT_STATUS         = 0x80,  /* payload: catcam_telemetry_t      */
    EVT_PIR_TRIGGERED  = 0x81,  /* payload: uint32 timestamp_ms     */
    EVT_STREAM_STARTED = 0x82,
    EVT_STREAM_STOPPED = 0x83,  /* payload: uint8 reason            */
    EVT_LOW_BATTERY    = 0x84,  /* payload: uint16 battery_mv       */
} cam_evt_t;

typedef enum {
    STATE_SLEEPING      = 0,
    STATE_WAKING        = 1,
    STATE_STREAMING     = 2,
    STATE_SHUTTING_DOWN = 3,
    STATE_FAULT         = 4,
} cam_state_t;

// (payload EVT_STREAM_STOPPED)
typedef enum {
    STOP_BY_COMMAND     = 0,  /* прийшов CMD_STOP_STREAM            */
    STOP_SESSION_LIMIT  = 1,  /* перевищено ліміт тривалості        */
    STOP_HEARTBEAT_LOST = 2,  /* втрачено heartbeat від VPS         */
    STOP_LOW_BATTERY    = 3,  /* критичний заряд                    */
} cam_stop_reason_t;

/* ---- Роздільності камери (payload CMD_SET_PARAMS) ---- */
typedef enum {
    RES_QVGA = 0,   /* 320x240  */
    RES_VGA  = 1,   /* 640x480  */
    RES_SVGA = 2,   /* 800x600  */
    RES_XGA  = 3,   /* 1024x768 */
} catcam_resolution_t;

/* ---- Параметри камери (CMD_SET_PARAMS) ---- */
typedef struct __attribute__((packed)) {
    uint8_t resolution;   /* catcam_resolution_t */
    uint8_t jpeg_quality; /* 10..30, менше = краще */
} catcam_params_t;

/* ---- Телеметрія (EVT_STATUS, відповідь на CMD_GET_STATUS) ---- */
typedef struct __attribute__((packed)) {
    uint16_t battery_mv;  /* напруга акумулятора, мВ           */
    uint16_t solar_mv;    /* напруга сонячної панелі, мВ       */
    uint8_t  state;       /* catcam_state_t                    */
    int16_t  pan_angle;   /* поточний кут повороту, deg        */
    int16_t  tilt_angle;  /* поточний кут нахилу, deg          */
    uint16_t pir_count;   /* лічильник спрацювань PIR          */
    int8_t   rssi;        /* рівень сигналу BLE-лінка, dBm     */
    uint32_t uptime_s;    /* час від перезавантаження, с       */
} catcam_telemetry_t;

/* ---- Межі сервоприводів (розділ 7 ТЗ) ---- */
#define SERVO_PAN_MIN    0
#define SERVO_PAN_MAX    180
#define SERVO_TILT_MIN   30
#define SERVO_TILT_MAX   150

/* ---- Розмір BLE-пакета ---- */
#define CATCAM_CMD_HEADER_SIZE   1
#define CATCAM_MAX_PAYLOAD_SIZE   8
#define CATCAM_MAX_PACKET_SIZE  (CATCAM_CMD_HEADER_SIZE + CATCAM_MAX_PAYLOAD_SIZE)