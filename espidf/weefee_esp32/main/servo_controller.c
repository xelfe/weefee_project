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

#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_system.h"
#include "driver/ledc.h"
#include "servo_controller.h"
#include "sdkconfig.h"
#include "rom/ets_sys.h"  // For ets_delay_us

static const char *TAG = "servo_controller";

// GPIOs used for servo control - values loaded from menuconfig
static const int servo_pins[SERVO_COUNT] = {
    CONFIG_SERVO_PIN_0, CONFIG_SERVO_PIN_1, CONFIG_SERVO_PIN_2, 
    CONFIG_SERVO_PIN_3, CONFIG_SERVO_PIN_4, CONFIG_SERVO_PIN_5,
    CONFIG_SERVO_PIN_6, CONFIG_SERVO_PIN_7, CONFIG_SERVO_PIN_8, 
    CONFIG_SERVO_PIN_9, CONFIG_SERVO_PIN_10, CONFIG_SERVO_PIN_11
};

// Servo values storage
static int servo_values[SERVO_COUNT] = {0};

// Initialize the LEDC timers and channels for PWM control
void setup_servos() {
    ESP_LOGI(TAG, "Initializing servos with pins: %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d",
             servo_pins[0], servo_pins[1], servo_pins[2], servo_pins[3], 
             servo_pins[4], servo_pins[5], servo_pins[6], servo_pins[7], 
             servo_pins[8], servo_pins[9], servo_pins[10], servo_pins[11]);
    
    ESP_LOGI(TAG, "Servo parameters: Freq=%dHz, Min pulse=%dus, Max pulse=%dus",
             SERVO_FREQ, SERVO_MIN_PULSEWIDTH, SERVO_MAX_PULSEWIDTH);
    
    // High-speed timer for first 8 servos
    ledc_timer_config_t timer_high = {
        .speed_mode = LEDC_HIGH_SPEED_MODE,
        .timer_num = LEDC_TIMER_HIGH,
        .duty_resolution = LEDC_TIMER_BIT,
        .freq_hz = SERVO_FREQ,
        .clk_cfg = LEDC_AUTO_CLK
    };
    ledc_timer_config(&timer_high);

    // Low-speed timer for remaining 4 servos
    ledc_timer_config_t timer_low = {
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .timer_num = LEDC_TIMER_LOW,
        .duty_resolution = LEDC_TIMER_BIT,
        .freq_hz = SERVO_FREQ,
        .clk_cfg = LEDC_AUTO_CLK
    };
    ledc_timer_config(&timer_low);

    // Configure LEDC channels
    for (int i = 0; i < SERVO_COUNT; i++) {
        ledc_channel_config_t channel = {
            .gpio_num = servo_pins[i],
            .speed_mode = (i < 8) ? LEDC_HIGH_SPEED_MODE : LEDC_LOW_SPEED_MODE,
            .channel = (i < 8) ? i : (i - 8),
            .timer_sel = (i < 8) ? LEDC_TIMER_HIGH : LEDC_TIMER_LOW,
            .duty = 0,
            .hpoint = 0
        };
        ledc_channel_config(&channel);
    }
    
    ESP_LOGI(TAG, "Servo initialization complete");
}

// Converts angle (0-180°) to PWM duty cycle
uint32_t angle_to_duty(int angle) {
    if (angle < 0) angle = 0;
    if (angle > 180) angle = 180;
    uint32_t us = SERVO_MIN_PULSEWIDTH + ((SERVO_MAX_PULSEWIDTH - SERVO_MIN_PULSEWIDTH) * angle / 180);
    uint32_t duty = (1 << LEDC_TIMER_BIT) * us / (1000000 / SERVO_FREQ);
    return duty;
}

// Applies servo angles with synchronized update
void apply_servo_positions(const int positions[SERVO_COUNT]) {
    // Reduced log level to DEBUG instead of INFO to avoid console pollution
    ESP_LOGD(TAG, "Applying servo positions: front legs (0-5), rear legs (6-11)");
    
    // Preprocessing: calculate duty cycles for all servos
    uint32_t duties[SERVO_COUNT];
    for (int i = 0; i < SERVO_COUNT; i++) {
        duties[i] = angle_to_duty(positions[i]);
        ESP_LOGD(TAG, "Servo %d set to %d° (duty=%lu)", i, positions[i], duties[i]);
    }
    
    // Apply duty values to all channels without updating them immediately
    for (int i = 0; i < SERVO_COUNT; i++) {
        ledc_set_duty(i < 8 ? LEDC_HIGH_SPEED_MODE : LEDC_LOW_SPEED_MODE,
                     i < 8 ? i : (i - 8), duties[i]);
    }
    
    // Update all channels of all legs simultaneously
    // Temporarily disable interrupts to ensure synchronization
    portDISABLE_INTERRUPTS();
    for (int i = 0; i < SERVO_COUNT; i++) {
        ledc_update_duty(i < 8 ? LEDC_HIGH_SPEED_MODE : LEDC_LOW_SPEED_MODE,
                        i < 8 ? i : (i - 8));
    }
    portENABLE_INTERRUPTS();
    
    // Reduced log level to DEBUG instead of INFO
    ESP_LOGD(TAG, "All servo positions updated simultaneously");
}

// Updates the servo values array
void set_servo_values(const int new_values[SERVO_COUNT]) {
    memcpy(servo_values, new_values, SERVO_COUNT * sizeof(int));
}

// Returns a pointer to the servo values array
const int* get_servo_values(void) {
    return servo_values;
}

// Returns a pointer to the servo pins array
const int* get_servo_pins(void) {
    return servo_pins;
}