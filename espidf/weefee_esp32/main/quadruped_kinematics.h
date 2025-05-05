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

#ifndef QUADRUPED_KINEMATICS_H
#define QUADRUPED_KINEMATICS_H

#include <stdint.h>
#include <stdlib.h>
#include <stdbool.h>
#include <math.h>
#include "esp_err.h"

// Structure representing a 3D position
typedef struct {
    float x;
    float y;
    float z;
} vec3_t;

// Structure for orientation in Euler angles
typedef struct {
    float roll;
    float pitch;
    float yaw;
} orientation_t;

// Structure representing a robot leg
typedef struct {
    // Servo IDs (coxa, femur, tibia)
    int servo_ids[3];
    
    // Leg dimensions (segment lengths)
    float coxa_length;
    float femur_length;
    float tibia_length;
    
    // Position of the leg mounting point relative to robot center
    vec3_t mounting_position;
    
    // Current foot position
    vec3_t foot_position;
    
    // Current joint angles
    float angles[3];
} leg_t;

// Constants for leg identification
#define LEG_FRONT_RIGHT   0
#define LEG_FRONT_LEFT    1
#define LEG_REAR_RIGHT    2
#define LEG_REAR_LEFT     3
#define LEG_COUNT         4

// Constants for joint identification
#define JOINT_COXA    0
#define JOINT_FEMUR   1
#define JOINT_TIBIA   2
#define JOINT_COUNT   3

// Kinematic function prototypes
esp_err_t inverse_kinematics(leg_t *leg, const vec3_t *target_pos, float angles_out[3]);
void forward_kinematics(const leg_t *leg, const float angles[3], vec3_t *position_out);
void init_kinematics(void);
void set_robot_dimensions(float body_length, float body_width, float coxa_length, float femur_length, float tibia_length);

#endif // QUADRUPED_KINEMATICS_H