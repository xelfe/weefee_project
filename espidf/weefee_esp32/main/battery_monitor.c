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
#include "esp_err.h"
#include "driver/i2c.h"
#include "battery_monitor.h"
#include "sdkconfig.h"

static const char *TAG = "battery_monitor";

// Debug logging macro - uses the same definition as in main.c
#ifdef CONFIG_DEBUG_LOGS_ENABLED
#define LOG_DEBUG(tag, format, ...) ESP_LOGI(tag, format, ##__VA_ARGS__)
#else
#define LOG_DEBUG(tag, format, ...) do {} while(0)
#endif

// I2C port used for communication
#define I2C_PORT            I2C_NUM_0

// Use the first INA219 address
#define INA219_I2C_ADDRESS  INA219_I2C_ADDRESS_1

// Default calibration value for INA219
#define DEFAULT_CALIB_VALUE 4096

#ifdef CONFIG_BAT_MONITOR_ENABLED
// Battery cell configuration from menuconfig
#define CELL_COUNT          CONFIG_BAT_MONITOR_CELL_COUNT
#define CELL_LOW_PCT        CONFIG_BAT_MONITOR_CELL_LOW_PCT
#define CELL_CRITICAL_PCT   CONFIG_BAT_MONITOR_CELL_CRITICAL_PCT

// Standard LiPo cell voltages - calculated based on cell chemistry
#define CELL_FULL_MV        4200  // Fully charged LiPo cell = 4.2V
#define CELL_EMPTY_MV       3000  // Fully discharged LiPo cell = 3.0V

// Calculate battery voltage thresholds (V)
#define DEFAULT_FULL_BATTERY      ((float)(CELL_FULL_MV * CELL_COUNT) / 1000.0f)
#define DEFAULT_EMPTY_BATTERY     ((float)(CELL_EMPTY_MV * CELL_COUNT) / 1000.0f)
#define DEFAULT_VOLTAGE_RANGE     (DEFAULT_FULL_BATTERY - DEFAULT_EMPTY_BATTERY)
#define DEFAULT_LOW_THRESHOLD     (DEFAULT_EMPTY_BATTERY + (DEFAULT_VOLTAGE_RANGE * CELL_LOW_PCT / 100.0f))
#define DEFAULT_CRITICAL_THRESHOLD (DEFAULT_EMPTY_BATTERY + (DEFAULT_VOLTAGE_RANGE * CELL_CRITICAL_PCT / 100.0f))
#else
// Default values when battery monitoring is disabled
#define DEFAULT_FULL_BATTERY      8.4f   // Default for 2S LiPo
#define DEFAULT_EMPTY_BATTERY     6.0f   // Default for 2S LiPo
#define DEFAULT_LOW_THRESHOLD     7.0f   // Default low threshold
#define DEFAULT_CRITICAL_THRESHOLD 6.5f  // Default critical threshold
#endif

// Battery monitoring state
static struct {
    battery_info_t info[1];
    float low_threshold;
    float critical_threshold;
    float full_voltage;
    float empty_voltage;
    bool initialized;
    bool monitoring_active;
    TaskHandle_t task_handle;
} battery_state = {
    .info = {{0}},
    .low_threshold = DEFAULT_LOW_THRESHOLD,
    .critical_threshold = DEFAULT_CRITICAL_THRESHOLD,
    .full_voltage = DEFAULT_FULL_BATTERY,
    .empty_voltage = DEFAULT_EMPTY_BATTERY,
    .initialized = false,
    .monitoring_active = false,
    .task_handle = NULL
};

// Forward declarations of helper functions
static esp_err_t ina219_write_register(uint8_t reg, uint16_t value);
static esp_err_t ina219_read_register(uint8_t reg, uint16_t *value);
static esp_err_t ina219_check_presence(void);
static void battery_monitor_task(void *pvParameters);

/**
 * @brief Initialize the battery monitor with INA219 sensor
 */
esp_err_t battery_monitor_init(int sda_pin, int scl_pin, uint32_t i2c_freq_hz) {
#ifndef CONFIG_BAT_MONITOR_ENABLED
    return ESP_ERR_NOT_SUPPORTED;
#else
    if (battery_state.initialized) {
        ESP_LOGW(TAG, "Battery monitor already initialized");
        return ESP_OK;
    }
    
    ESP_LOGI(TAG, "Initializing battery monitor with INA219 sensor");
    LOG_DEBUG(TAG, "I2C configuration: SDA=%d, SCL=%d, Freq=%lu Hz", sda_pin, scl_pin, i2c_freq_hz);
    LOG_DEBUG(TAG, "Battery configuration: %d cells, Full voltage: %.2fV, Empty voltage: %.2fV", 
             CELL_COUNT, DEFAULT_FULL_BATTERY, DEFAULT_EMPTY_BATTERY);
    LOG_DEBUG(TAG, "Battery thresholds: Low: %.2fV (%.0f%%), Critical: %.2fV (%.0f%%)",
             DEFAULT_LOW_THRESHOLD, (float)CELL_LOW_PCT, DEFAULT_CRITICAL_THRESHOLD, (float)CELL_CRITICAL_PCT);

    // Configure I2C
    i2c_config_t i2c_config = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = sda_pin,
        .scl_io_num = scl_pin,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = i2c_freq_hz
    };

    esp_err_t ret = i2c_param_config(I2C_PORT, &i2c_config);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "I2C parameter configuration failed: %s", esp_err_to_name(ret));
        return ret;
    }

    ret = i2c_driver_install(I2C_PORT, I2C_MODE_MASTER, 0, 0, 0);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "I2C driver installation failed: %s", esp_err_to_name(ret));
        return ret;
    }

    // Check INA219 presence
    ret = ina219_check_presence();
    if (ret != ESP_OK) {
        i2c_driver_delete(I2C_PORT);
        return ret;
    }

    // Configure INA219 with default calibration
    ret = battery_monitor_configure(DEFAULT_CALIB_VALUE);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "INA219 configuration failed: %s", esp_err_to_name(ret));
        i2c_driver_delete(I2C_PORT);
        return ret;
    }

    // Initialize battery state
    battery_state.initialized = true;
    battery_state.info[0].status = BATTERY_UNKNOWN;

    ESP_LOGI(TAG, "Battery monitor initialized successfully");
    return ESP_OK;
#endif
}

/**
 * @brief Configure the INA219 sensor
 */
esp_err_t battery_monitor_configure(uint16_t calibration_value) {
#ifndef CONFIG_BAT_MONITOR_ENABLED
    return ESP_ERR_NOT_SUPPORTED;
#else
    esp_err_t ret;

    // Reset the device first
    uint16_t config = 0x8000;  // Bit 15 set to 1 for reset
    ret = ina219_write_register(INA219_REG_CONFIG, config);
    if (ret != ESP_OK) {
        return ret;
    }
    vTaskDelay(pdMS_TO_TICKS(5)); // Short delay after reset

    // Configure for 32V range, 320mV shunt, 12-bit ADC resolution (both bus and shunt)
    // 0x399F = 0b0011 1001 1001 1111
    // Bits 15-13: 0b001 = BRNG (Bus Voltage Range): 32V
    // Bits 12-11: 0b11 = PG (PGA Gain): 320mV
    // Bits 10-7:  0b0011 = BADC (Bus ADC Resolution): 12-bit, 532μs conversion time
    // Bits 6-3:   0b0011 = SADC (Shunt ADC Resolution): 12-bit, 532μs conversion time
    // Bits 2-0:   0b111 = Mode: Shunt and bus, continuous
    config = 0x399F;
    ret = ina219_write_register(INA219_REG_CONFIG, config);
    if (ret != ESP_OK) {
        return ret;
    }

    // Set calibration register
    ret = ina219_write_register(INA219_REG_CALIB, calibration_value);
    if (ret != ESP_OK) {
        return ret;
    }

    LOG_DEBUG(TAG, "INA219 configured with calibration value: %u", calibration_value);
    return ESP_OK;
#endif
}

/**
 * @brief Read the current battery information
 */
esp_err_t battery_monitor_read(battery_info_t *info) {
#ifndef CONFIG_BAT_MONITOR_ENABLED
    // Fill with default values when monitoring is disabled
    if (info != NULL) {
        info->voltage = DEFAULT_FULL_BATTERY;
        info->current = 0.0f;
        info->power = 0.0f;
        info->remaining_pct = 100.0f;
        info->status = BATTERY_OK;
        
        // Store in battery state
        memcpy(&battery_state.info[0], info, sizeof(battery_info_t));
    }
    
    return ESP_ERR_NOT_SUPPORTED;
#else
    if (!battery_state.initialized) {
        ESP_LOGE(TAG, "Battery monitor not initialized");
        return ESP_ERR_INVALID_STATE;
    }

    if (info == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    uint16_t bus_voltage_reg, current_reg, power_reg;
    float shunt_voltage_mv, bus_voltage_v, current_ma, power_mw;

    // Read bus voltage register
    esp_err_t ret = ina219_read_register(INA219_REG_BUS_V, &bus_voltage_reg);
    if (ret != ESP_OK) {
        return ret;
    }

    // Read current register
    ret = ina219_read_register(INA219_REG_CURRENT, &current_reg);
    if (ret != ESP_OK) {
        return ret;
    }

    // Read power register
    ret = ina219_read_register(INA219_REG_POWER, &power_reg);
    if (ret != ESP_OK) {
        return ret;
    }

    // Convert register values to physical units
    // Bus voltage is in 4mV units, bits 0-2 are flags (not part of measurement)
    bus_voltage_v = ((bus_voltage_reg >> 3) * 4.0f) / 1000.0f;
    
    // Current and power values depend on calibration
    // For the default calibration of 4096, each LSB is 0.1mA for current
    current_ma = (int16_t)current_reg * 0.1f;
    
    // For power, each LSB is 2mW with the default calibration
    power_mw = power_reg * 2.0f;

    // Calculate remaining battery percentage
    float remaining = 0.0f;
    if (bus_voltage_v >= battery_state.empty_voltage) {
        remaining = (bus_voltage_v - battery_state.empty_voltage) / 
                   (battery_state.full_voltage - battery_state.empty_voltage) * 100.0f;
        
        // Clamp to 0-100%
        if (remaining > 100.0f) remaining = 100.0f;
        if (remaining < 0.0f) remaining = 0.0f;
    }

    // Determine battery status based on voltage thresholds
    battery_status_t status;
    if (bus_voltage_v <= battery_state.critical_threshold) {
        status = BATTERY_CRITICAL;
    } else if (bus_voltage_v <= battery_state.low_threshold) {
        status = BATTERY_LOW;
    } else {
        status = BATTERY_OK;
    }

    // Update battery info
    info->voltage = bus_voltage_v;
    info->current = current_ma;
    info->power = power_mw;
    info->remaining_pct = remaining;
    info->status = status;

    // Store latest values in battery state
    memcpy(&battery_state.info[0], info, sizeof(battery_info_t));

    ESP_LOGD(TAG, "Battery: %.2fV, %.1fmA, %.1fmW, %.1f%%, status=%d", 
             info->voltage, info->current, info->power, info->remaining_pct, info->status);

    return ESP_OK;
#endif
}

/**
 * @brief Get the current battery status
 */
battery_status_t battery_monitor_get_status(void) {
#ifndef CONFIG_BAT_MONITOR_ENABLED
    return BATTERY_OK;  // Always return OK when monitoring is disabled
#else
    if (!battery_state.initialized) {
        return BATTERY_UNKNOWN;
    }
    return battery_state.info[0].status;
#endif
}

/**
 * @brief Start battery monitoring task
 */
esp_err_t battery_monitor_start(uint32_t interval_ms) {
#ifndef CONFIG_BAT_MONITOR_ENABLED
    return ESP_ERR_NOT_SUPPORTED;
#else
    if (!battery_state.initialized) {
        ESP_LOGE(TAG, "Battery monitor not initialized");
        return ESP_ERR_INVALID_STATE;
    }

    if (battery_state.monitoring_active) {
        LOG_DEBUG(TAG, "Battery monitoring already active");
        return ESP_OK;
    }

    // Create battery monitoring task
    ESP_LOGI(TAG, "Starting battery monitoring task (interval: %lu ms)", interval_ms);
    BaseType_t ret = xTaskCreate(
        battery_monitor_task,
        "battery_mon",
        2048,  // Stack size
        (void*)interval_ms,
        3,     // Priority
        &battery_state.task_handle
    );

    if (ret != pdPASS) {
        ESP_LOGE(TAG, "Failed to create battery monitoring task");
        return ESP_FAIL;
    }

    battery_state.monitoring_active = true;
    return ESP_OK;
#endif
}

/**
 * @brief Stop battery monitoring task
 */
esp_err_t battery_monitor_stop(void) {
#ifndef CONFIG_BAT_MONITOR_ENABLED
    return ESP_ERR_NOT_SUPPORTED;
#else
    if (!battery_state.monitoring_active) {
        return ESP_OK;
    }

    if (battery_state.task_handle != NULL) {
        LOG_DEBUG(TAG, "Stopping battery monitoring task");
        vTaskDelete(battery_state.task_handle);
        battery_state.task_handle = NULL;
    }

    battery_state.monitoring_active = false;
    ESP_LOGI(TAG, "Battery monitoring stopped");
    return ESP_OK;
#endif
}

/**
 * @brief Get the last battery information
 */
esp_err_t battery_monitor_get_info(battery_info_t *info) {
    if (info == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

#ifndef CONFIG_BAT_MONITOR_ENABLED
    // Return default values
    info->voltage = DEFAULT_FULL_BATTERY;
    info->current = 0.0f;
    info->power = 0.0f;
    info->remaining_pct = 100.0f;
    info->status = BATTERY_OK;
    return ESP_OK;
#else
    if (!battery_state.initialized) {
        ESP_LOGE(TAG, "Battery monitor not initialized");
        return ESP_ERR_INVALID_STATE;
    }

    // Copy last read battery info
    memcpy(info, &battery_state.info[0], sizeof(battery_info_t));
    return ESP_OK;
#endif
}

// Battery monitoring task that periodically reads battery info
static void battery_monitor_task(void *pvParameters) {
    uint32_t interval_ms = (uint32_t)pvParameters;
    uint32_t update_count = 0;
    
    // Log once at startup
    ESP_LOGI(TAG, "Battery monitoring task started");

    while (1) {
        // Read battery info
        battery_info_t info;
        esp_err_t ret = battery_monitor_read(&info);
        
        if (ret == ESP_OK) {
            // Update status based on voltage thresholds
            if (info.voltage < battery_state.critical_threshold) {
                info.status = BATTERY_CRITICAL;
                // Always log critical battery status
                ESP_LOGW(TAG, "BATTERY CRITICAL: %.2fV (%.1f%%)", info.voltage, info.remaining_pct);
            } else if (info.voltage < battery_state.low_threshold) {
                info.status = BATTERY_LOW;
                // Log low battery only every 10 updates to avoid log spam
                if (update_count % 10 == 0) {
                    ESP_LOGW(TAG, "Battery low: %.2fV (%.1f%%)", info.voltage, info.remaining_pct);
                }
            } else {
                info.status = BATTERY_OK;
                // Log normal battery status less frequently (every 30 updates)
                if (update_count % 30 == 0) {
                    LOG_DEBUG(TAG, "Battery status: %.2fV (%.1f%%), %.2fmA, %.2fmW", 
                             info.voltage, info.remaining_pct, info.current, info.power);
                }
            }
            
            // Store updated info
            memcpy(&battery_state.info[0], &info, sizeof(battery_info_t));
        } else {
            // Log read errors, but not too often
            if (update_count % 20 == 0) {
                ESP_LOGW(TAG, "Failed to read battery info: %s", esp_err_to_name(ret));
            }
        }
        
        update_count++;
        vTaskDelay(pdMS_TO_TICKS(interval_ms));
    }
}

/**
 * @brief Set the battery thresholds for status levels
 */
esp_err_t battery_monitor_set_thresholds(float low_threshold, float critical_threshold) {
#ifndef CONFIG_BAT_MONITOR_ENABLED
    return ESP_ERR_NOT_SUPPORTED;
#else
    if (low_threshold <= critical_threshold) {
        ESP_LOGE(TAG, "Invalid thresholds: low must be higher than critical");
        return ESP_ERR_INVALID_ARG;
    }

    battery_state.low_threshold = low_threshold;
    battery_state.critical_threshold = critical_threshold;
    
    LOG_DEBUG(TAG, "Battery thresholds set: low=%.2fV, critical=%.2fV", 
             low_threshold, critical_threshold);
    return ESP_OK;
#endif
}

/**
 * @brief Write a 16-bit value to an INA219 register
 */
static esp_err_t ina219_write_register(uint8_t reg, uint16_t value) {
    uint8_t write_buf[3];
    write_buf[0] = reg;
    write_buf[1] = (value >> 8) & 0xFF;  // MSB first (big endian)
    write_buf[2] = value & 0xFF;         // LSB
    
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (INA219_I2C_ADDRESS << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write(cmd, write_buf, sizeof(write_buf), true);
    i2c_master_stop(cmd);
    
    esp_err_t ret = i2c_master_cmd_begin(I2C_PORT, cmd, pdMS_TO_TICKS(100));
    i2c_cmd_link_delete(cmd);
    
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "Failed to write to INA219 register 0x%02x: %s", reg, esp_err_to_name(ret));
    }
    
    return ret;
}

/**
 * @brief Read a 16-bit value from an INA219 register
 */
static esp_err_t ina219_read_register(uint8_t reg, uint16_t *value) {
    if (value == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    
    // Send register address
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (INA219_I2C_ADDRESS << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, reg, true);
    
    // Restart and read data
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (INA219_I2C_ADDRESS << 1) | I2C_MASTER_READ, true);
    
    uint8_t read_buf[2];
    i2c_master_read_byte(cmd, &read_buf[0], I2C_MASTER_ACK);     // MSB
    i2c_master_read_byte(cmd, &read_buf[1], I2C_MASTER_NACK);    // LSB
    i2c_master_stop(cmd);
    
    esp_err_t ret = i2c_master_cmd_begin(I2C_PORT, cmd, pdMS_TO_TICKS(100));
    i2c_cmd_link_delete(cmd);
    
    if (ret == ESP_OK) {
        *value = (read_buf[0] << 8) | read_buf[1];  // MSB first (big endian)
    } else {
        ESP_LOGW(TAG, "Failed to read from INA219 register 0x%02x: %s", reg, esp_err_to_name(ret));
    }
    
    return ret;
}

/**
 * @brief Check if INA219 sensor is present on I2C bus
 */
static esp_err_t ina219_check_presence(void) {
    uint16_t value;
    esp_err_t ret = ina219_read_register(INA219_REG_CONFIG, &value);
    
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "INA219 sensor not found on I2C bus (addr: 0x%02X)", INA219_I2C_ADDRESS);
        return ESP_ERR_NOT_FOUND;
    }
    
    LOG_DEBUG(TAG, "INA219 sensor found at address 0x%02X, config reg: 0x%04X", 
             INA219_I2C_ADDRESS, value);
    return ESP_OK;
}