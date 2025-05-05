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

#ifndef QUADRUPED_ROBOT_H
#define QUADRUPED_ROBOT_H

#include <stdint.h>
#include <stdlib.h>
#include <stdbool.h>
#include "esp_err.h"
#include "quadruped_kinematics.h"
#include "servo_controller.h"

// Structure representing the robot state
typedef struct {
    // Robot legs
    leg_t legs[LEG_COUNT];
    
    // Body position and orientation
    vec3_t body_position;
    orientation_t body_orientation;
    
    // Gait configuration
    float gait_height;
    float step_length;
    float step_height;
    
    // Robot state
    bool is_standing;
    bool is_walking;
    float walk_cycle_progress;
} quadruped_t;

// Gait type definitions
typedef enum {
    GAIT_STAND = 0,
    GAIT_WALK = 1,
    GAIT_TROT = 2,
    GAIT_PACE = 3
} gait_type_t;

// Function prototypes
void robot_init(void);
esp_err_t robot_set_body_position(const vec3_t *position);
esp_err_t robot_set_body_orientation(const orientation_t *orientation);
esp_err_t robot_set_leg_position(int leg_index, const vec3_t *position);
esp_err_t robot_stand(float height);
esp_err_t robot_sit(void);
esp_err_t robot_start_gait(gait_type_t gait, float speed);
esp_err_t robot_stop_gait(void);
void robot_update(void);
void robot_map_angles_to_servos(void);

#endif // QUADRUPED_ROBOT_H