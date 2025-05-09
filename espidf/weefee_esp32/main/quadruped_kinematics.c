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
    // Variables for calculations
    float coxa_angle, femur_angle, tibia_angle;
    float leg_x = target_pos->x - leg->mounting_position.x;
    float leg_y = target_pos->y - leg->mounting_position.y;
    float leg_z = target_pos->z - leg->mounting_position.z;
    
    // Horizontal distance from leg base to foot
    float L = sqrtf(leg_x * leg_x + leg_y * leg_y);
    
    // Distance for coxa (first segment)
    float L_coxa = leg->coxa_length;
    
    // Distance for femur/tibia joints
    float L_femur_tibia = sqrtf((L - L_coxa) * (L - L_coxa) + leg_z * leg_z);
    
    // Check if position is reachable
    if (L_femur_tibia > (leg->femur_length + leg->tibia_length)) {
        ESP_LOGW(TAG, "Target position out of reach: %.2f > %.2f", 
                L_femur_tibia, (leg->femur_length + leg->tibia_length));
        return ESP_ERR_INVALID_ARG;
    }
    
    // Calculate coxa angle (horizontal rotation)
    coxa_angle = atan2f(leg_y, leg_x) * 180.0f / M_PI;
    
    // Adjust positions for femur and tibia calculations
    float L2 = L - L_coxa;
    
    // Calculate femur and tibia angles using law of cosines
    float a = leg->femur_length;
    float b = leg->tibia_length;
    float c = L_femur_tibia;
    
    // Angle between horizontal and the line from femur to foot
    float gamma = atan2f(leg_z, L2) * 180.0f / M_PI;
    
    // Angle between femur-foot line and femur segment
    float alpha = acosf((a*a + c*c - b*b) / (2.0f * a * c)) * 180.0f / M_PI;
    
    // Angle between femur and tibia segments
    float beta = acosf((a*a + b*b - c*c) / (2.0f * a * b)) * 180.0f / M_PI;
    
    // Final angle calculations (according to robot convention)
    femur_angle = 90.0f - (gamma + alpha);
    tibia_angle = 180.0f - beta;
    
    // Assign results
    angles_out[JOINT_COXA] = coxa_angle;
    angles_out[JOINT_FEMUR] = femur_angle;
    angles_out[JOINT_TIBIA] = tibia_angle;
    
    // Using static variable to reduce the frequency of IK result logging
    static uint32_t last_ik_log = 0;
    uint32_t current_time = xTaskGetTickCount() * portTICK_PERIOD_MS;
    
    // Log only once every 2 seconds to reduce overhead
    if (current_time - last_ik_log > 2000) {
        LOG_DEBUG(TAG, "IK result: [%.2f, %.2f, %.2f] for position [%.2f, %.2f, %.2f]",
                 coxa_angle, femur_angle, tibia_angle, target_pos->x, target_pos->y, target_pos->z);
        last_ik_log = current_time;
    }
    
    return ESP_OK;
}

void forward_kinematics(const leg_t *leg, const float angles[3], vec3_t *position_out) {
    // Convert angles to radians
    float coxa_rad = angles[JOINT_COXA] * M_PI / 180.0f;
    float femur_rad = angles[JOINT_FEMUR] * M_PI / 180.0f;
    
    // Position after coxa rotation
    float x_coxa = leg->coxa_length * cosf(coxa_rad);
    float y_coxa = leg->coxa_length * sinf(coxa_rad);
    float z_coxa = 0.0f;
    
    // Adjustment for femur angle
    float femur_rad_adjusted = (90.0f * M_PI / 180.0f) - femur_rad;
    float x_femur = leg->femur_length * cosf(femur_rad_adjusted) * cosf(coxa_rad);
    float y_femur = leg->femur_length * cosf(femur_rad_adjusted) * sinf(coxa_rad);
    float z_femur = -leg->femur_length * sinf(femur_rad_adjusted);
    
    // Adjustment for tibia angle - using angles[JOINT_TIBIA] directly instead of tibia_rad
    float tibia_rad_adjusted = ((180.0f - angles[JOINT_TIBIA]) * M_PI / 180.0f) + femur_rad_adjusted;
    float x_tibia = leg->tibia_length * cosf(tibia_rad_adjusted) * cosf(coxa_rad);
    float y_tibia = leg->tibia_length * cosf(tibia_rad_adjusted) * sinf(coxa_rad);
    float z_tibia = -leg->tibia_length * sinf(tibia_rad_adjusted);
    
    // Final foot position
    position_out->x = leg->mounting_position.x + x_coxa + x_femur + x_tibia;
    position_out->y = leg->mounting_position.y + y_coxa + y_femur + y_tibia;
    position_out->z = leg->mounting_position.z + z_coxa + z_femur + z_tibia;
    
    // Using static variable to reduce the frequency of FK result logging
    static uint32_t last_fk_log = 0;
    uint32_t current_time = xTaskGetTickCount() * portTICK_PERIOD_MS;
    
    // Log only once every 2 seconds
    if (current_time - last_fk_log > 2000) {
        LOG_DEBUG(TAG, "FK result: [%.2f, %.2f, %.2f] for angles [%.2f, %.2f, %.2f]",
                 position_out->x, position_out->y, position_out->z, 
                 angles[JOINT_COXA], angles[JOINT_FEMUR], angles[JOINT_TIBIA]);
        last_fk_log = current_time;
    }
}