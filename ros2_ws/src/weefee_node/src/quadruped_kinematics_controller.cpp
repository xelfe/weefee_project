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
#include "std_msgs/msg/int32_multi_array.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "quadruped_inverse_kinematics.h"
#include <iostream>
#include <vector>
#include <string>
#include <cmath>

using namespace std::chrono_literals;

/**
 * @brief 3D Vector structure - represents positions in 3D space
 */
struct Vec3 {
    float x = 0.0f;
    float y = 0.0f;
    float z = 0.0f;
};

/**
 * @brief Orientation structure - represents orientation in Euler angles
 */
struct Orientation {
    float roll = 0.0f;
    float pitch = 0.0f;
    float yaw = 0.0f;
};

/**
 * @brief Leg structure - represents a single robot leg with its joints
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

/**
 * @brief QuadrupedKinematicsController - Advanced ROS2 node for quadruped control
 * 
 * This node implements forward and inverse kinematics for a quadruped robot,
 * allowing precise control of leg positions and robot movements.
 */
class QuadrupedKinematicsController : public rclcpp::Node
{
public:
    QuadrupedKinematicsController()
    : Node("quadruped_kinematics_controller")
    {
        // Publishers
        command_pub_ = this->create_publisher<std_msgs::msg::String>("robot_command", 10);
        servo_angles_pub_ = this->create_publisher<std_msgs::msg::Int32MultiArray>("servo_angles", 10);
        
        // Subscribers
        body_pose_sub_ = this->create_subscription<geometry_msgs::msg::Pose>(
            "body_pose", 10, std::bind(&QuadrupedKinematicsController::body_pose_callback, this, std::placeholders::_1));
        
        gait_control_sub_ = this->create_subscription<std_msgs::msg::String>(
            "gait_control", 10, std::bind(&QuadrupedKinematicsController::gait_control_callback, this, std::placeholders::_1));
        
        // Initialize robot configuration
        initialize_robot();
        
        // Control loop timer (runs at 50Hz)
        control_timer_ = this->create_wall_timer(20ms, std::bind(&QuadrupedKinematicsController::control_loop, this));
        
        RCLCPP_INFO(this->get_logger(), "Quadruped kinematics controller initialized.");
    }

private:
    /**
     * @brief Initialize robot configuration
     */
    void initialize_robot()
    {
        // Robot dimensions
        body_length_ = 120.0f;
        body_width_ = 90.0f;
        
        // Configure legs
        legs_.resize(4);
        
        // Set mounting positions for each leg (FR, FL, RR, RL)
        legs_[0].mounting_position = {body_length_/2, body_width_/2, 0};   // Front Right
        legs_[1].mounting_position = {body_length_/2, -body_width_/2, 0};  // Front Left
        legs_[2].mounting_position = {-body_length_/2, body_width_/2, 0};  // Rear Right
        legs_[3].mounting_position = {-body_length_/2, -body_width_/2, 0}; // Rear Left
        
        // Set default foot positions (under mounting positions at standing height)
        for (int i = 0; i < 4; i++) {
            legs_[i].foot_position = legs_[i].mounting_position;
            legs_[i].foot_position.z = -150.0f; // Default standing height
        }
        
        // Initialize body position and orientation
        body_position_ = {0.0f, 0.0f, 0.0f};
        body_orientation_ = {0.0f, 0.0f, 0.0f};
        
        // Initial gait parameters
        gait_type_ = "stand";
        gait_speed_ = 1.0f;
        step_height_ = 30.0f;
        step_length_ = 60.0f;
        
        // Initial state
        is_standing_ = false;
        is_walking_ = false;
        walk_cycle_progress_ = 0.0f;
        
        RCLCPP_INFO(this->get_logger(), "Robot model initialized with %d legs", (int)legs_.size());
    }
    
    /**
     * @brief Handle body pose commands
     */
    void body_pose_callback(const geometry_msgs::msg::Pose::SharedPtr msg)
    {
        // Extract position
        body_position_.x = msg->position.x;
        body_position_.y = msg->position.y;
        body_position_.z = msg->position.z;
        
        // Convert quaternion to Euler angles
        // Simple conversion assuming small angles
        body_orientation_.roll = 0.0f;  // Would need quaternion conversion
        body_orientation_.pitch = 0.0f; // Would need quaternion conversion
        body_orientation_.yaw = 0.0f;   // Would need quaternion conversion
        
        RCLCPP_INFO(this->get_logger(), "New body pose: pos=[%.2f, %.2f, %.2f]",
                   body_position_.x, body_position_.y, body_position_.z);
        
        // Send command to robot
        send_position_command();
    }
    
    /**
     * @brief Handle gait control commands
     */
    void gait_control_callback(const std_msgs::msg::String::SharedPtr msg)
    {
        std::string command = msg->data;
        
        if (command == "stand") {
            stand_command();
        }
        else if (command == "sit") {
            sit_command();
        }
        else if (command.substr(0, 4) == "walk") {
            // Extract speed if provided
            float speed = 1.0f;
            if (command.length() > 5) {
                try {
                    speed = std::stof(command.substr(5));
                } catch (...) {
                    RCLCPP_WARN(this->get_logger(), "Invalid speed value, using default 1.0");
                }
            }
            walk_command(speed);
        }
        else if (command == "trot") {
            trot_command();
        }
        else if (command == "stop") {
            stop_command();
        }
        else {
            RCLCPP_WARN(this->get_logger(), "Unknown command: %s", command.c_str());
        }
    }
    
    /**
     * @brief Main control loop running at 50Hz
     */
    void control_loop()
    {
        if (is_walking_) {
            // Update walk cycle
            walk_cycle_progress_ += 0.01f * gait_speed_;
            if (walk_cycle_progress_ >= 1.0f) {
                walk_cycle_progress_ = 0.0f;
            }
            
            // Generate leg positions for current gait
            if (gait_type_ == "walk") {
                calculate_walking_gait();
            }
            else if (gait_type_ == "trot") {
                calculate_trotting_gait();
            }
            
            // Apply the calculated leg positions with inverse kinematics
            update_leg_positions();
        }
    }
    
    /**
     * @brief Calculate foot positions for walking gait
     */
    void calculate_walking_gait()
    {
        // Walking gait sequence: FR, RR, FL, RL
        // Each leg moves one after another
        
        // Define phase offsets for each leg (0.0 to 1.0)
        float phase_offsets[4] = {0.0f, 0.25f, 0.5f, 0.75f};
        
        for (int i = 0; i < 4; i++) {
            // Calculate phase for this leg
            float phase = walk_cycle_progress_ - phase_offsets[i];
            if (phase < 0.0f) phase += 1.0f;
            
            // Default position (standing)
            Vec3 foot_pos = legs_[i].mounting_position;
            foot_pos.z = -150.0f;
            
            if (phase < 0.5f) {
                // Swing phase (foot in air)
                float swing_progress = phase / 0.5f; // 0.0 to 1.0
                
                // X movement (front to back)
                foot_pos.x += step_length_ * (0.5f - swing_progress);
                
                // Z movement (up and down)
                float height_factor = sin(swing_progress * M_PI);
                foot_pos.z += step_height_ * height_factor;
            } else {
                // Stance phase (foot on ground)
                float stance_progress = (phase - 0.5f) / 0.5f; // 0.0 to 1.0
                
                // X movement (back to front)
                foot_pos.x += step_length_ * (stance_progress - 0.5f);
            }
            
            // Store calculated position
            target_foot_positions_[i] = foot_pos;
        }
    }
    
    /**
     * @brief Calculate foot positions for trotting gait
     */
    void calculate_trotting_gait()
    {
        // Trotting gait: diagonal pairs move together
        // FR+RL, then FL+RR
        
        // Define diagonal pairs
        int diagonal_pairs[2][2] = {{0, 3}, {1, 2}}; // FR+RL, FL+RR
        
        for (int pair = 0; pair < 2; pair++) {
            // Calculate phase for this pair
            float phase = walk_cycle_progress_ - 0.5f * pair;
            if (phase < 0.0f) phase += 1.0f;
            
            for (int leg = 0; leg < 2; leg++) {
                int leg_idx = diagonal_pairs[pair][leg];
                
                // Default position (standing)
                Vec3 foot_pos = legs_[leg_idx].mounting_position;
                foot_pos.z = -150.0f;
                
                if (phase < 0.5f) {
                    // Swing phase (foot in air)
                    float swing_progress = phase / 0.5f; // 0.0 to 1.0
                    
                    // X movement (front to back)
                    foot_pos.x += step_length_ * (0.5f - swing_progress);
                    
                    // Z movement (up and down)
                    float height_factor = sin(swing_progress * M_PI);
                    foot_pos.z += step_height_ * height_factor;
                } else {
                    // Stance phase (foot on ground)
                    float stance_progress = (phase - 0.5f) / 0.5f; // 0.0 to 1.0
                    
                    // X movement (back to front)
                    foot_pos.x += step_length_ * (stance_progress - 0.5f);
                }
                
                // Store calculated position
                target_foot_positions_[leg_idx] = foot_pos;
            }
        }
    }
    
    /**
     * @brief Update leg positions based on target foot positions
     */
    void update_leg_positions()
    {
        // Create kinematics engine
        QuadrupedInverseKinematics kinematics;
        kinematics.set_robot_dimensions(body_length_, body_width_, 30.0f, 70.0f, 90.0f);

        // Prepare message to hold servo angles
        auto servo_angles_msg = std_msgs::msg::Int32MultiArray();
        servo_angles_msg.data.resize(12); // 4 legs × 3 servos
        
        // Calculate angles for each leg using inverse kinematics
        for (int i = 0; i < 4; i++) {
            // Apply body position/orientation adjustments to foot position
            Vec3 adjusted_target = target_foot_positions_[i];
            
            // TODO: Add body orientation transformation calculations here
            // For now just apply position offset
            adjusted_target.x += body_position_.x;
            adjusted_target.y += body_position_.y;
            adjusted_target.z += body_position_.z;
            
            // Calculate joint angles using inverse kinematics
            float angles[3] = {0.0f};
            if (kinematics.inverse_kinematics(i, adjusted_target, angles)) {
                // Store the calculated angles
                for (int j = 0; j < 3; j++) {
                    legs_[i].angles[j] = angles[j];
                    
                    // Convert to integer and add to message
                    int servo_idx = i * 3 + j;
                    servo_angles_msg.data[servo_idx] = static_cast<int>(round(angles[j]));
                }
            } else {
                RCLCPP_WARN(this->get_logger(), "Leg %d: Target position unreachable", i);
            }
        }
        
        // Publish servo angles
        servo_angles_pub_->publish(servo_angles_msg);
        
        RCLCPP_DEBUG(this->get_logger(), "Published servo angles");
    }
    
    /**
     * @brief Sends a stand command to the robot
     */
    void stand_command()
    {
        auto msg = std_msgs::msg::String();
        msg.data = "stand";
        command_pub_->publish(msg);
        
        // Update local state
        gait_type_ = "stand";
        is_standing_ = true;
        is_walking_ = false;
        
        RCLCPP_INFO(this->get_logger(), "Command sent: Stand");
    }
    
    /**
     * @brief Sends a sit command to the robot
     */
    void sit_command()
    {
        auto msg = std_msgs::msg::String();
        msg.data = "sit";
        command_pub_->publish(msg);
        
        // Update local state
        gait_type_ = "sit";
        is_standing_ = false;
        is_walking_ = false;
        
        RCLCPP_INFO(this->get_logger(), "Command sent: Sit");
    }
    
    /**
     * @brief Sends a walk command with specified speed
     */
    void walk_command(float speed)
    {
        auto msg = std_msgs::msg::String();
        msg.data = "walk " + std::to_string(speed);
        command_pub_->publish(msg);
        
        // Update local state
        gait_type_ = "walk";
        gait_speed_ = speed;
        is_walking_ = true;
        
        RCLCPP_INFO(this->get_logger(), "Command sent: Walk at speed %.2f", speed);
    }
    
    /**
     * @brief Sends a trot command to the robot
     */
    void trot_command()
    {
        auto msg = std_msgs::msg::String();
        msg.data = "trot";
        command_pub_->publish(msg);
        
        // Update local state
        gait_type_ = "trot";
        is_walking_ = true;
        
        RCLCPP_INFO(this->get_logger(), "Command sent: Trot");
    }
    
    /**
     * @brief Stops any current movement
     */
    void stop_command()
    {
        auto msg = std_msgs::msg::String();
        msg.data = "stop";
        command_pub_->publish(msg);
        
        // Update local state
        is_walking_ = false;
        
        RCLCPP_INFO(this->get_logger(), "Command sent: Stop");
    }
    
    /**
     * @brief Sends the current body position and orientation to the robot
     */
    void send_position_command()
    {
        auto msg = std_msgs::msg::String();
        msg.data = "position " + 
                   std::to_string(body_position_.x) + " " +
                   std::to_string(body_position_.y) + " " +
                   std::to_string(body_position_.z);
        command_pub_->publish(msg);
        
        auto ori_msg = std_msgs::msg::String();
        ori_msg.data = "orientation " + 
                      std::to_string(body_orientation_.roll) + " " +
                      std::to_string(body_orientation_.pitch) + " " +
                      std::to_string(body_orientation_.yaw);
        command_pub_->publish(ori_msg);
        
        RCLCPP_INFO(this->get_logger(), "Position and orientation commands sent");
    }

    // Publishers
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr command_pub_;
    rclcpp::Publisher<std_msgs::msg::Int32MultiArray>::SharedPtr servo_angles_pub_;
    
    // Subscribers
    rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr body_pose_sub_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr gait_control_sub_;
    
    // Timer
    rclcpp::TimerBase::SharedPtr control_timer_;
    
    // Robot model
    float body_length_;
    float body_width_;
    std::vector<Leg> legs_;
    Vec3 body_position_;
    Orientation body_orientation_;
    
    // Target positions for feet
    Vec3 target_foot_positions_[4];
    
    // Gait parameters
    std::string gait_type_;
    float gait_speed_;
    float step_height_;
    float step_length_;
    
    // State flags
    bool is_standing_;
    bool is_walking_;
    float walk_cycle_progress_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<QuadrupedKinematicsController>());
    rclcpp::shutdown();
    return 0;
}