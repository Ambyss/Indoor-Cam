#pragma once

#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"
#include "BLE_protocol.h"

esp_err_t cam_servo_init(void);

esp_err_t cam_servo_power_on(void);
void      cam_servo_power_off(void);
bool      cam_servo_is_powered(void);

void      cam_servo_move(int16_t pan, int16_t tilt);
void      cam_servo_set_pan(int16_t deg);
void      cam_servo_set_tilt(int16_t deg);
void      cam_servo_home(void);

// Joystick: -100..100 % of max speed.
// Must be repeated at least every 500 ms, otherwise stops.
void      cam_servo_velocity(int8_t vx, int8_t vy);

int16_t   cam_servo_get_pan(void);
int16_t   cam_servo_get_tilt(void);
bool      cam_servo_is_moving(void);

esp_err_t cam_servo_set_cal(const cam_servo_cal_t *cal);
void      cam_servo_get_cal(cam_servo_cal_t *out);
void      cam_servo_save(void);