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

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include <string>
#include "quadruped_common.h"

/**
 * @brief Implementation of quadruped robot inverse kinematics functions
 * 
 * This file provides functions for calculating joint angles from desired foot positions,
 * which is essential for controlling the movement of a quadruped robot.
 */

// Use namespace for common structures
using namespace weefee;

/**
 * @brief QuadrupedInverseKinematics - Provides inverse & forward kinematics for quadruped robots
 * 
 * This class implements the mathematical calculations needed to convert between:
 * - Foot positions and joint angles (inverse kinematics)
 * - Joint angles and foot positions (forward kinematics)
 */
class QuadrupedInverseKinematics {
public:
    QuadrupedInverseKinematics() {
        // Initialize default values
        body_length_ = 120.0f;
        body_width_ = 90.0f;
        
        // Create default leg configuration
        initialize_legs();
    }
    
    /**
     * @brief Set up robot dimensions
     * 
     * @param body_length Length of the robot body
     * @param body_width Width of the robot body
     * @param coxa_length Length of the coxa (hip) segment
     * @param femur_length Length of the femur (thigh) segment
     * @param tibia_length Length of the tibia (shin) segment
     */
    void set_robot_dimensions(float body_length, float body_width, 
                             float coxa_length, float femur_length, float tibia_length) {
        body_length_ = body_length;
        body_width_ = body_width;
        default_coxa_length_ = coxa_length;
        default_femur_length_ = femur_length;
        default_tibia_length_ = tibia_length;
        
        // Update leg dimensions
        for (int i = 0; i < LEG_COUNT; i++) {
            legs_[i].coxa_length = coxa_length;
            legs_[i].femur_length = femur_length;
            legs_[i].tibia_length = tibia_length;
        }
    }
    
    /**
     * @brief Initialize leg configuration
     */
    void initialize_legs() {
        // Set mounting positions for each leg (FR, FL, RR, RL)
        legs_[LEG_FRONT_RIGHT].mounting_position = {body_length_/2, body_width_/2, 0};
        legs_[LEG_FRONT_LEFT].mounting_position = {body_length_/2, -body_width_/2, 0};
        legs_[LEG_REAR_RIGHT].mounting_position = {-body_length_/2, body_width_/2, 0};
        legs_[LEG_REAR_LEFT].mounting_position = {-body_length_/2, -body_width_/2, 0};
        
        // Set default foot positions (under mounting positions at standing height)
        for (int i = 0; i < LEG_COUNT; i++) {
            legs_[i].foot_position = legs_[i].mounting_position;
            legs_[i].foot_position.z = -150.0f; // Default standing height
        }
    }
    
    /**
     * @brief Calculate inverse kinematics for a leg
     * 
     * @param leg_index Index of the leg (0-3)
     * @param target_pos Target foot position
     * @param angles_out Output array for the calculated angles
     * @return true if calculation succeeded, false if target is unreachable
     */
    bool inverse_kinematics(int leg_index, const Vec3 &target_pos, float angles_out[3]) {
        if (leg_index < 0 || leg_index >= LEG_COUNT) {
            return false;
        }
        
        Leg &leg = legs_[leg_index];
        
        // Variables for calculations
        float coxa_angle, femur_angle, tibia_angle;
        
        // Calculate leg-relative position
        float leg_x = target_pos.x - leg.mounting_position.x;
        float leg_y = target_pos.y - leg.mounting_position.y;
        float leg_z = target_pos.z - leg.mounting_position.z;
        
        // Horizontal distance from leg base to foot
        float L = sqrtf(leg_x * leg_x + leg_y * leg_y);
        
        // Distance for coxa (first segment)
        float L_coxa = leg.coxa_length;
        
        // Distance for femur/tibia joints
        float L_femur_tibia = sqrtf((L - L_coxa) * (L - L_coxa) + leg_z * leg_z);
        
        // Check if position is reachable
        if (L_femur_tibia > (leg.femur_length + leg.tibia_length)) {
            return false; // Target position out of reach
        }
        
        // Calculate coxa angle (horizontal rotation)
        coxa_angle = atan2f(leg_y, leg_x) * 180.0f / M_PI;
        
        // Adjust positions for femur and tibia calculations
        float L2 = L - L_coxa;
        
        // Calculate femur and tibia angles using law of cosines
        float a = leg.femur_length;
        float b = leg.tibia_length;
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
        
        return true;
    }
    
    /**
     * @brief Calculate forward kinematics for a leg
     * 
     * @param leg_index Index of the leg (0-3)
     * @param angles Array of joint angles (coxa, femur, tibia)
     * @param position_out Output position
     */
    void forward_kinematics(int leg_index, const float angles[3], Vec3 &position_out) {
        if (leg_index < 0 || leg_index >= LEG_COUNT) {
            return;
        }
        
        Leg &leg = legs_[leg_index];
        
        // Convert angles to radians
        float coxa_rad = angles[JOINT_COXA] * M_PI / 180.0f;
        float femur_rad = angles[JOINT_FEMUR] * M_PI / 180.0f;
        // Tibia angle is used in calculation below with tibia_rad_adjusted
        
        // Position after coxa rotation
        float x_coxa = leg.coxa_length * cosf(coxa_rad);
        float y_coxa = leg.coxa_length * sinf(coxa_rad);
        float z_coxa = 0.0f;
        
        // Adjustment for femur angle
        float femur_rad_adjusted = (90.0f * M_PI / 180.0f) - femur_rad;
        float x_femur = leg.femur_length * cosf(femur_rad_adjusted) * cosf(coxa_rad);
        float y_femur = leg.femur_length * cosf(femur_rad_adjusted) * sinf(coxa_rad);
        float z_femur = -leg.femur_length * sinf(femur_rad_adjusted);
        
        // Adjustment for tibia angle
        float tibia_rad_adjusted = ((180.0f - angles[JOINT_TIBIA]) * M_PI / 180.0f) + femur_rad_adjusted;
        float x_tibia = leg.tibia_length * cosf(tibia_rad_adjusted) * cosf(coxa_rad);
        float y_tibia = leg.tibia_length * cosf(tibia_rad_adjusted) * sinf(coxa_rad);
        float z_tibia = -leg.tibia_length * sinf(tibia_rad_adjusted);
        
        // Final foot position
        position_out.x = leg.mounting_position.x + x_coxa + x_femur + x_tibia;
        position_out.y = leg.mounting_position.y + y_coxa + y_femur + y_tibia;
        position_out.z = leg.mounting_position.z + z_coxa + z_femur + z_tibia;
    }
    
    /**
     * @brief Get a reference to the specified leg
     * 
     * @param leg_index Index of the leg (0-3)
     * @return Reference to the leg or nullptr if invalid index
     */
    Leg* get_leg(int leg_index) {
        if (leg_index < 0 || leg_index >= LEG_COUNT) {
            return nullptr;
        }
        return &legs_[leg_index];
    }

private:
    // Robot dimensions
    float body_length_ = 0.0f;
    float body_width_ = 0.0f;
    float default_coxa_length_ = 0.0f;
    float default_femur_length_ = 0.0f;
    float default_tibia_length_ = 0.0f;
    
    // Leg configurations
    Leg legs_[LEG_COUNT];
};