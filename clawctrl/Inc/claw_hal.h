#ifndef __CLAW_HAL_H
#define __CLAW_HAL_H

#include "stdint.h"

uint8_t read_up_ls();
uint8_t read_down_ls();

uint8_t read_btn1();
uint8_t read_btn2();

void set_user_leds(uint8_t on1, uint8_t on2, uint8_t on3);

void set_btn1_led(uint8_t on);
void set_btn2_led(uint8_t on);

void beep(uint32_t times, uint32_t period);

#endif