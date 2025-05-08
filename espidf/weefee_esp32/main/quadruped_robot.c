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
#include "quadruped_robot.h"
#include "quadruped_kinematics.h"
#include "servo_controller.h"
#include "sdkconfig.h"

static const char *TAG = "robot";

// Robot instance
static quadruped_t robot;

// Servo mapping: [leg_index][joint_index]
static const int servo_mapping[LEG_COUNT][JOINT_COUNT] = {
    {0, 1, 2},     // Front right leg - coxa, femur, tibia
    {3, 4, 5},     // Front left leg
    {6, 7, 8},     // Rear right leg - rétabli au mappage d'origine
    {9, 10, 11}    // Rear left leg - rétabli au mappage d'origine
};

// Robot initialization
void robot_init(void) {
    ESP_LOGI(TAG, "Initializing quadruped robot");
    
    // Initialize servos
    setup_servos();
    
    // Initialize kinematics
    init_kinematics();
    
    // Robot dimensions are now configured through menuconfig (as integers in millimeters)
    int body_length = CONFIG_ROBOT_BODY_LENGTH;
    int body_width = CONFIG_ROBOT_BODY_WIDTH;
    int coxa_length = CONFIG_DEFAULT_COXA_LENGTH;
    int femur_length = CONFIG_DEFAULT_FEMUR_LENGTH;
    int tibia_length = CONFIG_DEFAULT_TIBIA_LENGTH;
    
    // Configure kinematics dimensions (keeping values in millimeters)
    set_robot_dimensions(body_length, body_width, coxa_length, femur_length, tibia_length);
    
    // Initialize legs
    // Leg mounting positions relative to body center (in millimeters)
    vec3_t mounting_positions[LEG_COUNT] = {
        {body_length/2,  body_width/2,  0},  // Front right
        {body_length/2, -body_width/2,  0},  // Front left
        {-body_length/2,  body_width/2, 0},  // Rear right
        {-body_length/2, -body_width/2, 0}   // Rear left
    };
    
    // Configure legs
    for (int i = 0; i < LEG_COUNT; i++) {
        for (int j = 0; j < JOINT_COUNT; j++) {
            robot.legs[i].servo_ids[j] = servo_mapping[i][j];
        }
        
        robot.legs[i].coxa_length = coxa_length;
        robot.legs[i].femur_length = femur_length;
        robot.legs[i].tibia_length = tibia_length;
        robot.legs[i].mounting_position = mounting_positions[i];
        
        // Default foot positions
        robot.legs[i].foot_position = mounting_positions[i];
        robot.legs[i].foot_position.z = -110.0f;  // Ground position
        
        // Initial joint angles - modified to match correct physical assembly position
        robot.legs[i].angles[JOINT_COXA] = 90.0f;
        robot.legs[i].angles[JOINT_FEMUR] = 45.0f;  // Changed from 90 to 45 degrees
        robot.legs[i].angles[JOINT_TIBIA] = 90.0f;
    }
    
    // Initial body position and orientation
    robot.body_position.x = 0.0f;
    robot.body_position.y = 0.0f;
    robot.body_position.z = 0.0f;
    
    robot.body_orientation.roll = 0.0f;
    robot.body_orientation.pitch = 0.0f;
    robot.body_orientation.yaw = 0.0f;
    
    // Gait configuration
    robot.gait_height = 150.0f;
    robot.step_length = 60.0f;
    robot.step_height = 30.0f;
    
    // Initial state
    robot.is_standing = false;
    robot.is_walking = false;
    robot.walk_cycle_progress = 0.0f;
    
    // Set and apply positions based on the configured joint angles
    int servo_positions[SERVO_COUNT];
    
    // Apply initial joint angles to physical servos
    for (int i = 0; i < LEG_COUNT; i++) {
        for (int j = 0; j < JOINT_COUNT; j++) {
            // Store angle in servo array based on the mapping
            servo_positions[robot.legs[i].servo_ids[j]] = (int)robot.legs[i].angles[j];
        }
    }
    
    set_servo_values(servo_positions);
    apply_servo_positions(servo_positions);
    
    ESP_LOGI(TAG, "Robot initialization complete with servos at configured angles (COXA:90°, FEMUR:45°, TIBIA:90°)");
}

// Update servo angles from calculated angles
void robot_map_angles_to_servos(void) {
    int servo_angles[SERVO_COUNT] = {0};
    
    for (int i = 0; i < LEG_COUNT; i++) {
        for (int j = 0; j < JOINT_COUNT; j++) {
            // Convert calculated angles to servo angles
            int servo_angle = (int)robot.legs[i].angles[j];
            
            // Safety limits
            if (servo_angle < 0) servo_angle = 0;
            if (servo_angle > 180) servo_angle = 180;
            
            // Store in servo array
            servo_angles[robot.legs[i].servo_ids[j]] = servo_angle;
        }
    }
    
    // Apply angles to servos
    set_servo_values(servo_angles);
    apply_servo_positions(servo_angles);
}

// Set robot body position
esp_err_t robot_set_body_position(const vec3_t *position) {
    if (position == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    
    // Update body position
    robot.body_position = *position;
    
    // Recalculate all leg positions and update servos
    robot_update();
    
    return ESP_OK;
}

// Set robot body orientation
esp_err_t robot_set_body_orientation(const orientation_t *orientation) {
    if (orientation == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    
    // Update body orientation
    robot.body_orientation = *orientation;
    
    // Recalculate all leg positions and update servos
    robot_update();
    
    return ESP_OK;
}

// Set position of a specific leg
esp_err_t robot_set_leg_position(int leg_index, const vec3_t *position) {
    if (leg_index < 0 || leg_index >= LEG_COUNT || position == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    
    // Calculate inverse kinematics to determine angles
    float angles[3];
    esp_err_t result = inverse_kinematics(&robot.legs[leg_index], position, angles);
    
    if (result == ESP_OK) {
        // Update leg angles
        for (int i = 0; i < JOINT_COUNT; i++) {
            robot.legs[leg_index].angles[i] = angles[i];
        }
        
        // Update foot position
        robot.legs[leg_index].foot_position = *position;
        
        // Update servos
        robot_map_angles_to_servos();
    }
    
    return result;
}

// Put robot in standing position
esp_err_t robot_stand(float height) {
    if (height <= 0) {
        height = robot.gait_height;
    }
    
    ESP_LOGI(TAG, "Robot standing up to height %.2f mm", height);
    
    // Body position
    vec3_t body_pos = {0, 0, 0};
    robot_set_body_position(&body_pos);
    
    // Body orientation
    orientation_t body_ori = {0, 0, 0};
    robot_set_body_orientation(&body_ori);
    
    // Position the legs
    for (int i = 0; i < LEG_COUNT; i++) {
        vec3_t foot_pos = robot.legs[i].mounting_position;
        foot_pos.z = -height;
        
        robot_set_leg_position(i, &foot_pos);
    }
    
    robot.is_standing = true;
    return ESP_OK;
}

// Put robot in sitting position
esp_err_t robot_sit(void) {
    ESP_LOGI(TAG, "Robot sitting down");
    
    // Body position
    vec3_t body_pos = {0, 0, -30.0f}; // Slight body lowering
    robot_set_body_position(&body_pos);
    
    // Body orientation
    orientation_t body_ori = {0, 0, 0};
    robot_set_body_orientation(&body_ori);
    
    // Position the legs
    for (int i = 0; i < LEG_COUNT; i++) {
        vec3_t foot_pos = robot.legs[i].mounting_position;
        
        // Front legs stay extended, rear legs fold
        if (i == LEG_FRONT_RIGHT || i == LEG_FRONT_LEFT) {
            foot_pos.z = -80.0f;
        } else {
            foot_pos.z = -50.0f;
            foot_pos.x -= 30.0f; // Fold rear legs
        }
        
        robot_set_leg_position(i, &foot_pos);
    }
    
    robot.is_standing = false;
    return ESP_OK;
}

// Start a specific gait
esp_err_t robot_start_gait(gait_type_t gait, float speed) {
    if (!robot.is_standing) {
        ESP_LOGW(TAG, "Cannot start gait, robot not standing");
        return ESP_ERR_INVALID_STATE;
    }
    
    ESP_LOGI(TAG, "Starting gait %d with speed %.2f", gait, speed);
    
    // Initialize walk cycle
    robot.is_walking = true;
    robot.walk_cycle_progress = 0.0f;
    
    // Gait implementation is done in robot_update
    
    return ESP_OK;
}

// Stop current gait
esp_err_t robot_stop_gait(void) {
    if (!robot.is_walking) {
        return ESP_OK;
    }
    
    ESP_LOGI(TAG, "Stopping gait");
    
    robot.is_walking = false;
    
    // Return to standing position
    return robot_stand(robot.gait_height);
}

// Update robot state
void robot_update(void) {
    if (robot.is_walking) {
        // Update walk cycle (implement according to chosen gait)
        // Simplified example for walking gait:
        
        // Advance cycle by 1%
        robot.walk_cycle_progress += 0.01f;
        if (robot.walk_cycle_progress >= 1.0f) {
            robot.walk_cycle_progress = 0.0f;
        }
        
        // Calculate foot positions according to walk phases
        // (Actual implementation would vary by gait type)
        
        // For now, do nothing
    }
    
    // Update servos from current state
    robot_map_angles_to_servos();
}