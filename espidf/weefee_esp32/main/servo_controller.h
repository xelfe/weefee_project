#ifndef SERVO_CONTROLLER_H
#define SERVO_CONTROLLER_H

#include <stdint.h>
#include <stdlib.h>
#include <stdbool.h>
#include "driver/ledc.h"
#include "esp_err.h"

// Constants for servo control
#define SERVO_COUNT 12
#define SERVO_MIN_PULSEWIDTH 500
#define SERVO_MAX_PULSEWIDTH 2500
#define SERVO_FREQ 50

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