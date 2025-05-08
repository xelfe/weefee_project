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

#ifndef BATTERY_MONITOR_H
#define BATTERY_MONITOR_H

#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"

// Number of batteries to monitor (only 1 supported currently)
#define BATTERY_COUNT     1

// INA219 I2C Addresses (default: 0x40 and 0x41)
#define INA219_I2C_ADDRESS_1    0x40
#define INA219_I2C_ADDRESS_2    0x41

// INA219 Register Addresses
#define INA219_REG_CONFIG     0x00
#define INA219_REG_SHUNT_V    0x01
#define INA219_REG_BUS_V      0x02
#define INA219_REG_POWER      0x03
#define INA219_REG_CURRENT    0x04
#define INA219_REG_CALIB      0x05

// Battery status enumeration
typedef enum {
    BATTERY_OK = 0,          // Battery level normal
    BATTERY_LOW = 1,         // Battery level low
    BATTERY_CRITICAL = 2,    // Battery level critical
    BATTERY_UNKNOWN = 3      // Battery status unknown
} battery_status_t;

// Battery information structure
typedef struct {
    float voltage;           // Current battery voltage in volts
    float current;           // Current consumption in milliamps
    float power;             // Power consumption in milliwatts
    float remaining_pct;     // Remaining battery percentage (0-100)
    battery_status_t status; // Current battery status
} battery_info_t;

/**
 * @brief Initialize the battery monitor with INA219 sensor
 * @param sda_pin GPIO pin for I2C SDA
 * @param scl_pin GPIO pin for I2C SCL
 * @param i2c_freq_hz I2C frequency in Hz
 * @return ESP_OK on success or an error code
 */
esp_err_t battery_monitor_init(int sda_pin, int scl_pin, uint32_t i2c_freq_hz);

/**
 * @brief Configure the INA219 sensor
 * @param calibration_value Calibration value
 * @return ESP_OK on success or an error code
 */
esp_err_t battery_monitor_configure(uint16_t calibration_value);

/**
 * @brief Read the current battery information
 * @param info Pointer to battery_info_t structure to store the data
 * @return ESP_OK on success or an error code
 */
esp_err_t battery_monitor_read(battery_info_t *info);

/**
 * @brief Get the current battery status
 * @return Current battery status
 */
battery_status_t battery_monitor_get_status(void);

/**
 * @brief Start a background task to monitor battery status
 * @param update_interval_ms Update interval in milliseconds
 * @return ESP_OK on success or an error code
 */
esp_err_t battery_monitor_start_task(uint32_t update_interval_ms);

/**
 * @brief Stop the battery monitoring task
 * @return ESP_OK on success
 */
esp_err_t battery_monitor_stop_task(void);

/**
 * @brief Set the battery thresholds for status levels
 * @param low_threshold Voltage threshold for BATTERY_LOW status
 * @param critical_threshold Voltage threshold for BATTERY_CRITICAL status
 * @return ESP_OK on success
 */
esp_err_t battery_monitor_set_thresholds(float low_threshold, float critical_threshold);

#endif // BATTERY_MONITOR_H