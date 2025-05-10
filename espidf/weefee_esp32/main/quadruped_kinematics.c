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
#include <math.h>
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "quadruped_kinematics.h"
#include "sdkconfig.h"

static const char *TAG = "kinematics";

// Conditional log macros to reduce log volume
#ifdef CONFIG_DEBUG_LOGS_ENABLED
#define LOG_DEBUG(tag, format, ...) ESP_LOGD(tag, format, ##__VA_ARGS__)
#else
#define LOG_DEBUG(tag, format, ...) 
#endif

// Constants for angle conversions
#define DEG_TO_RAD (M_PI / 180.0f)
#define RAD_TO_DEG (180.0f / M_PI)

// Robot dimensions (initialized from menuconfig, keeping values in millimeters)
static float robot_body_length = CONFIG_ROBOT_BODY_LENGTH;  // millimeters
static float robot_body_width = CONFIG_ROBOT_BODY_WIDTH;    // millimeters
static float default_coxa_length = CONFIG_DEFAULT_COXA_LENGTH;    // millimeters
static float default_femur_length = CONFIG_DEFAULT_FEMUR_LENGTH;  // millimeters
static float default_tibia_length = CONFIG_DEFAULT_TIBIA_LENGTH;  // millimeters

void init_kinematics(void) {
    ESP_LOGI(TAG, "Initializing quadruped kinematics");
    LOG_DEBUG(TAG, "Robot dimensions (from menuconfig) - Body: %.2f x %.2f, Leg segments: %.2f, %.2f, %.2f",
             robot_body_length, robot_body_width, default_coxa_length, default_femur_length, default_tibia_length);
}

void set_robot_dimensions(float body_length, float body_width, float coxa_length, float femur_length, float tibia_length) {
    robot_body_length = body_length;
    robot_body_width = body_width;
    default_coxa_length = coxa_length;
    default_femur_length = femur_length;
    default_tibia_length = tibia_length;
    
    LOG_DEBUG(TAG, "Robot dimensions set - Body: %.2f x %.2f, Leg segments: %.2f, %.2f, %.2f",
             body_length, body_width, coxa_length, femur_length, tibia_length);
}

esp_err_t inverse_kinematics(leg_t *leg, const vec3_t *target_pos, float angles_out[3]) {
    // This function is now a stub since calculations are done by ROS2
    // Maintain the function signature for compatibility with existing code
    ESP_LOGW(TAG, "Warning: ESP32 inverse_kinematics called but computations are now handled by ROS2");
    
    // Return pre-calculated values or default values to avoid crashes
    // in case this function is still called somewhere in the code
    angles_out[JOINT_COXA] = 90.0f;
    angles_out[JOINT_FEMUR] = 45.0f;
    angles_out[JOINT_TIBIA] = 90.0f;
    
    return ESP_OK;
}

void forward_kinematics(const leg_t *leg, const float angles[3], vec3_t *position_out) {
    // This function is now a stub since calculations are done by ROS2
    // Maintain the function signature for compatibility with existing code
    ESP_LOGW(TAG, "Warning: ESP32 forward_kinematics called but computations are now handled by ROS2");
    
    // Return simplified position calculation or last known position
    // Just estimate a position based on leg mounting and default length
    position_out->x = leg->mounting_position.x;
    position_out->y = leg->mounting_position.y;
    position_out->z = leg->mounting_position.z - 150.0f; // Approximate standing height
    
    LOG_DEBUG(TAG, "FK (simplified) result: [%.2f, %.2f, %.2f]",
              position_out->x, position_out->y, position_out->z);
}