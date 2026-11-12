#pragma once

#include <stdint.h>
#include <stdbool.h>

// DFS + automatic light sleep
void cam_power_init(void);

void cam_power_hold(void);      // block light sleep
void cam_power_release(void);   // allow light sleep

// mV, stub (TODO: ADC)
uint16_t cam_power_battery_mv(void);