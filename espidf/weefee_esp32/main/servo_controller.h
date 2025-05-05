/*
 * MIT License
 *
 * Copyright (c) 2025 xelfe (plapensee@lapensee-electronique.ca)
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

#ifndef SERVO_CONTROLLER_H
#define SERVO_CONTROLLER_H

#include <stdint.h>
#include <stdlib.h>
#include <stdbool.h>
#include "driver/ledc.h"
#include "esp_err.h"
#include "sdkconfig.h"

// Constants for servo control
#define SERVO_COUNT 12
#define SERVO_MIN_PULSEWIDTH CONFIG_SERVO_MIN_PULSEWIDTH
#define SERVO_MAX_PULSEWIDTH CONFIG_SERVO_MAX_PULSEWIDTH
#define SERVO_FREQ CONFIG_SERVO_FREQ

#define LEDC_TIMER_BIT LEDC_TIMER_13_BIT
#define LEDC_TIMER_HIGH LEDC_TIMER_0
#define LEDC_TIMER_LOW LEDC_TIMER_1

// Function prototypes
void setup_servos(void);
uint32_t angle_to_duty(int angle);
void apply_servo_positions(const int positions[SERVO_COUNT]);
void set_servo_values(const int new_values[SERVO_COUNT]);
const int* get_servo_values(void);
const int* get_servo_pins(void);

#endif // SERVO_CONTROLLER_H