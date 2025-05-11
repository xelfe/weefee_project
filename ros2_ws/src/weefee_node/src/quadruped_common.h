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

#ifndef QUADRUPED_COMMON_H
#define QUADRUPED_COMMON_H

#include <cmath>

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

namespace weefee {

/**
 * @brief 3D Vector structure - represents positions in 3D space
 * 
 * This structure corresponds to vec3_t in the ESP32 code
 */
struct Vec3 {
    float x = 0.0f;
    float y = 0.0f;
    float z = 0.0f;
};

/**
 * @brief Orientation structure - represents orientation in Euler angles
 * 
 * This structure corresponds to orientation_t in the ESP32 code
 */
struct Orientation {
    float roll = 0.0f;
    float pitch = 0.0f;
    float yaw = 0.0f;
};

/**
 * @brief Leg structure - represents a single robot leg with its dimensions and state
 * 
 * This structure corresponds to leg_t in the ESP32 code
 */
struct Leg {
    // Joint angles in degrees
    float angles[3] = {90.0f, 90.0f, 90.0f};
    
    // Mounting position relative to body center
    Vec3 mounting_position;
    
    // Current foot position
    Vec3 foot_position;
    
    // Leg dimensions
    float coxa_length = 30.0f;
    float femur_length = 70.0f;
    float tibia_length = 90.0f;
};

} // namespace weefee

#endif // QUADRUPED_COMMON_H
