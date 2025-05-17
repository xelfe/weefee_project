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
#include "geometry_msgs/msg/pose.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "urdf/model.h"
#include <string>
#include <vector>
#include "quadruped_common.h"

using namespace std::chrono_literals;
using namespace weefee;

/**
 * @brief QuadrupedModelPublisher - Publishes URDF model and joint states for RViz
 * 
 * This node complements the existing QuadrupedVisualizer by creating a proper
 * URDF model representation of the 12-DOF quadruped robot and publishing joint
 * states based on servo angles.
 */
class QuadrupedModelPublisher : public rclcpp::Node
{
public:
    QuadrupedModelPublisher()
    : Node("quadruped_model_publisher")
    {
        // Publishers
        joint_state_pub_ = this->create_publisher<sensor_msgs::msg::JointState>("joint_states", 10);

        // Subscribers
        servo_sub_ = this->create_subscription<std_msgs::msg::Int32MultiArray>(
            "servo_angles", 10, std::bind(&QuadrupedModelPublisher::servo_callback, this, std::placeholders::_1));

        body_pose_sub_ = this->create_subscription<geometry_msgs::msg::Pose>(
            "robot_pose", 10, std::bind(&QuadrupedModelPublisher::body_pose_callback, this, std::placeholders::_1));
            
        // Subscribe to robot_command topic to respond to high-level commands (calibrate, sit, stand)
        // and direct servo commands for proper visualization synchronization with actual robot state
        command_sub_ = this->create_subscription<std_msgs::msg::String>(
            "robot_command", 10, std::bind(&QuadrupedModelPublisher::command_callback, this, std::placeholders::_1));

        // Update timer - 30Hz
        update_timer_ = this->create_wall_timer(33ms, std::bind(&QuadrupedModelPublisher::publish_joint_states, this));

        // Initialize robot dimensions from sdkconfig.defaults values
        // These would typically be loaded from parameters
        body_length_ = 120.0;  // mm
        body_width_ = 90.0;    // mm
        coxa_length_ = 30.0;   // mm
        femur_length_ = 70.0;  // mm
        tibia_length_ = 90.0;  // mm

        // Initialize joint angles
        for (int i = 0; i < 12; i++) {
            // Default positions based on calibration: 90,45,90
            if (i % 3 == 1) {
                joint_angles_[i] = 45.0;  // Femur joints
            } else {
                joint_angles_[i] = 90.0;  // Coxa and tibia joints
            }
        }

        RCLCPP_INFO(this->get_logger(), "Quadruped model publisher initialized");
    }

private:
    /**
     * @brief Process servo angle updates
     */
    void servo_callback(const std_msgs::msg::Int32MultiArray::SharedPtr msg)
    {
        if (msg->data.size() == 12) {
            // Update joint angles from servo positions
            for (int i = 0; i < 12; i++) {
                joint_angles_[i] = static_cast<double>(msg->data[i]);
            }
            
            // Convert angles from servo space to joint space if needed
            convert_servo_to_joint_angles();
        }
    }

    /**
     * @brief Process body pose updates
     */
    void body_pose_callback(const geometry_msgs::msg::Pose::SharedPtr msg)
    {
        body_position_x_ = msg->position.x;
        body_position_y_ = msg->position.y;
        body_position_z_ = msg->position.z;
        
        // Optional: extract orientation from quaternion
    }
    
    /**
     * @brief Process commands from robot_command topic for RViz visualization
     * 
     * This callback processes both high-level commands (calibrate, sit, stand)
     * and direct servo commands ("servo:90,45,90,...") to ensure the RViz
     * visualization accurately reflects the robot's physical state even when
     * commands bypass the servo_angles topic.
     */
    void command_callback(const std_msgs::msg::String::SharedPtr msg)
    {
        // Check for direct servo commands first
        if (msg->data.find("servo:") == 0) {
            // Format is "servo:angle1,angle2,..."
            std::string angles_str = msg->data.substr(6); // Skip "servo:"
            std::vector<int> servo_angles;
            
            // Parse angles
            size_t pos = 0;
            std::string token;
            while ((pos = angles_str.find(",")) != std::string::npos) {
                token = angles_str.substr(0, pos);
                servo_angles.push_back(std::stoi(token));
                angles_str.erase(0, pos + 1);
            }
            // Add the last angle
            if (!angles_str.empty()) {
                servo_angles.push_back(std::stoi(angles_str));
            }
            
            if (servo_angles.size() == 12) {
                // Update joint angles from parsed servo positions
                for (int i = 0; i < 12; i++) {
                    joint_angles_[i] = static_cast<double>(servo_angles[i]);
                }
                
                // Convert angles from servo space to joint space
                convert_servo_to_joint_angles();
                RCLCPP_INFO(this->get_logger(), "Updated model from direct servo command");
            }
        }
        // Handle high-level command: calibrate
        else if (msg->data == "calibrate") {
            // Set calibration positions (90,45,90 pattern for all legs)
            // This represents the robot's standard calibration/assembly position
            std::vector<double> calibration_angles = {90, 45, 90, 90, 45, 90, 90, 45, 90, 90, 45, 90};
            for (int i = 0; i < 12; i++) {
                joint_angles_[i] = calibration_angles[i];
            }
            convert_servo_to_joint_angles();
            RCLCPP_INFO(this->get_logger(), "Updated model with calibration position");
        }
        else if (msg->data == "sit") {
            // Set sit positions
            std::vector<double> sit_angles = {
                90, 130, 60,  // Front Right (FR)
                90, 130, 60,  // Front Left (FL)
                90, 90, 90,   // Rear Right (RR)
                90, 90, 90    // Rear Left (RL)
            };
            for (int i = 0; i < 12; i++) {
                joint_angles_[i] = sit_angles[i];
            }
            convert_servo_to_joint_angles();
            RCLCPP_INFO(this->get_logger(), "Updated model with sit position");
        }
        else if (msg->data == "stand") {
            // Set stand positions
            std::vector<double> stand_angles = {
                90, 45, 90,  // Front Right
                90, 45, 90,  // Front Left
                90, 45, 90,  // Rear Right
                90, 45, 90   // Rear Left
            };
            for (int i = 0; i < 12; i++) {
                joint_angles_[i] = stand_angles[i];
            }
            convert_servo_to_joint_angles();
            RCLCPP_INFO(this->get_logger(), "Updated model with stand position");
        }
    }

    /**
     * @brief Converts servo angles to joint angles considering hardware configuration
     * 
     * This accounts for servo mounting orientation and any offsets needed
     */
    void convert_servo_to_joint_angles()
    {
        // For each leg
        for (int leg = 0; leg < 4; leg++) {
            // Coxa joint (servo angles might need inversion)
            int coxa_idx = leg * 3 + 0;
            // Adjusting coxa angle based on your firmware code that inverts these
            joint_angles_[coxa_idx] = 180.0 - joint_angles_[coxa_idx];
            
            // Femur and tibia adjustments if needed
            // int femur_idx = leg * 3 + 1;
            // int tibia_idx = leg * 3 + 2;
        }
    }

    /**
     * @brief Publish joint states for the robot model
     */
    void publish_joint_states()
    {
        auto joint_state_msg = sensor_msgs::msg::JointState();
        joint_state_msg.header.stamp = this->now();

        // Define joint names
        joint_state_msg.name = {
            "body_to_fr_coxa", "fr_coxa_to_femur", "fr_femur_to_tibia",
            "body_to_fl_coxa", "fl_coxa_to_femur", "fl_femur_to_tibia",
            "body_to_rr_coxa", "rr_coxa_to_femur", "rr_femur_to_tibia",
            "body_to_rl_coxa", "rl_coxa_to_femur", "rl_femur_to_tibia"
        };

        // Set joint positions from stored angles (in radians)
        joint_state_msg.position.resize(12);
        for (int i = 0; i < 12; i++) {
            joint_state_msg.position[i] = joint_angles_[i] * M_PI / 180.0;
        }

        // Publish joint states
        joint_state_pub_->publish(joint_state_msg);
    }

    // Publishers
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_pub_;

    // Subscribers
    rclcpp::Subscription<std_msgs::msg::Int32MultiArray>::SharedPtr servo_sub_;  // For servo_angles topic
    rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr body_pose_sub_;    // For robot_pose topic
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr command_sub_;         // For robot_command topic

    // Timer
    rclcpp::TimerBase::SharedPtr update_timer_;

    // Robot dimensions
    double body_length_;
    double body_width_;
    double coxa_length_;
    double femur_length_;
    double tibia_length_;

    // Joint state tracking
    std::array<double, 12> joint_angles_;
    
    // Body position and orientation
    double body_position_x_ = 0.0;
    double body_position_y_ = 0.0;
    double body_position_z_ = 0.0;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<QuadrupedModelPublisher>());
    rclcpp::shutdown();
    return 0;
}