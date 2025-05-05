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
#include "std_msgs/msg/bool.hpp"
#include <iostream>
#include <vector>
#include <string>

using namespace std::chrono_literals;

/**
 * @brief QuadrupedController - ROS2 node for controlling a quadruped robot
 * 
 * This class implements a ROS2 node that provides control interfaces for a 
 * quadruped robot running on an ESP32 microcontroller. It supports:
 * - Direct servo angle control
 * - Body position and orientation control
 * - Gait selection and control
 * - Robot status monitoring
 */
class QuadrupedController : public rclcpp::Node
{
public:
    QuadrupedController()
    : Node("quadruped_controller")
    {
        // Publishers
        servo_pub_ = this->create_publisher<std_msgs::msg::Int32MultiArray>("servo_angles", 10);
        command_pub_ = this->create_publisher<std_msgs::msg::String>("robot_command", 10);
        
        // Subscribers
        status_sub_ = this->create_subscription<std_msgs::msg::String>(
            "robot_status", 10, std::bind(&QuadrupedController::status_callback, this, std::placeholders::_1));
        
        // Service clients can be added later if needed
        
        // Initialize all servos to neutral position
        auto servo_msg = std_msgs::msg::Int32MultiArray();
        servo_msg.data = {90, 90, 90, 90, 90, 90, 90, 90, 90, 90, 90, 90};  // 12 servos at neutral position
        servo_pub_->publish(servo_msg);
        
        // Create timers for UI updates (simulating user commands for demonstration)
        ui_timer_ = this->create_wall_timer(1000ms, std::bind(&QuadrupedController::ui_update, this));
        
        RCLCPP_INFO(this->get_logger(), "Quadruped controller initialized.");
        
        // Initialize state
        current_state_ = "INIT";
        demo_counter_ = 0;
    }

private:
    /**
     * @brief Process status messages from the robot
     */
    void status_callback(const std_msgs::msg::String::SharedPtr msg)
    {
        RCLCPP_INFO(this->get_logger(), "Robot status: %s", msg->data.c_str());
        // Update internal state based on status
        robot_status_ = msg->data;
    }
    
    /**
     * @brief Sends a command to the robot to stand up
     */
    void stand_command()
    {
        auto msg = std_msgs::msg::String();
        msg.data = "stand";
        command_pub_->publish(msg);
        RCLCPP_INFO(this->get_logger(), "Command sent: Stand");
    }
    
    /**
     * @brief Sends a command to the robot to sit down
     */
    void sit_command()
    {
        auto msg = std_msgs::msg::String();
        msg.data = "sit";
        command_pub_->publish(msg);
        RCLCPP_INFO(this->get_logger(), "Command sent: Sit");
    }
    
    /**
     * @brief Sends a walking command with specified speed
     * @param speed Walking speed (default: 1.0)
     */
    void walk_command(float speed = 1.0f)
    {
        auto msg = std_msgs::msg::String();
        msg.data = "walk " + std::to_string(speed);
        command_pub_->publish(msg);
        RCLCPP_INFO(this->get_logger(), "Command sent: Walk at speed %.2f", speed);
    }
    
    /**
     * @brief Sends a trot gait command
     */
    void trot_command()
    {
        auto msg = std_msgs::msg::String();
        msg.data = "trot";
        command_pub_->publish(msg);
        RCLCPP_INFO(this->get_logger(), "Command sent: Trot");
    }
    
    /**
     * @brief Sends command to stop any active movement
     */
    void stop_command()
    {
        auto msg = std_msgs::msg::String();
        msg.data = "stop";
        command_pub_->publish(msg);
        RCLCPP_INFO(this->get_logger(), "Command sent: Stop");
    }
    
    /**
     * @brief Sets the body position of the robot
     * @param x X position (forward/backward)
     * @param y Y position (left/right)
     * @param z Z position (up/down)
     */
    void set_body_position(float x, float y, float z)
    {
        auto msg = std_msgs::msg::String();
        msg.data = "position " + std::to_string(x) + " " + 
                   std::to_string(y) + " " + std::to_string(z);
        command_pub_->publish(msg);
        RCLCPP_INFO(this->get_logger(), "Command sent: Position [%.2f, %.2f, %.2f]", x, y, z);
    }
    
    /**
     * @brief Sets the body orientation of the robot
     * @param roll Roll angle (x-axis rotation)
     * @param pitch Pitch angle (y-axis rotation)
     * @param yaw Yaw angle (z-axis rotation)
     */
    void set_body_orientation(float roll, float pitch, float yaw)
    {
        auto msg = std_msgs::msg::String();
        msg.data = "orientation " + std::to_string(roll) + " " + 
                   std::to_string(pitch) + " " + std::to_string(yaw);
        command_pub_->publish(msg);
        RCLCPP_INFO(this->get_logger(), "Command sent: Orientation [%.2f, %.2f, %.2f]", 
                    roll, pitch, yaw);
    }
    
    /**
     * @brief UI update timer callback - demonstrates different robot commands
     * 
     * This function cycles through various robot commands to showcase the 
     * capabilities of the quadruped robot.
     */
    void ui_update()
    {
        // Simple demo sequence
        switch (demo_counter_)
        {
            case 0:
                // Start by standing up
                current_state_ = "STANDING";
                stand_command();
                break;
            case 5:
                // Adjust body position slightly forward
                set_body_position(20.0, 0.0, 0.0);
                break;
            case 7:
                // Adjust body orientation (pitch down slightly)
                set_body_orientation(0.0, 10.0, 0.0);
                break;
            case 9:
                // Reset position and orientation
                set_body_position(0.0, 0.0, 0.0);
                set_body_orientation(0.0, 0.0, 0.0);
                break;
            case 11:
                // Start walking slowly
                current_state_ = "WALKING";
                walk_command(0.5);
                break;
            case 15:
                // Increase walking speed
                walk_command(1.0);
                break;
            case 19:
                // Switch to trot gait
                current_state_ = "TROTTING";
                trot_command();
                break;
            case 23:
                // Stop movement
                current_state_ = "STOPPED";
                stop_command();
                break;
            case 25:
                // Sit down
                current_state_ = "SITTING";
                sit_command();
                break;
            case 30:
                // Reset counter to restart demo
                demo_counter_ = -1;
                break;
        }
        
        demo_counter_++;
        
        // Log the current state
        RCLCPP_INFO(this->get_logger(), "Robot state: %s (Demo step: %d)", 
                   current_state_.c_str(), demo_counter_);
    }

    // Publishers
    rclcpp::Publisher<std_msgs::msg::Int32MultiArray>::SharedPtr servo_pub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr command_pub_;
    
    // Subscribers
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr status_sub_;
    
    // Timers
    rclcpp::TimerBase::SharedPtr ui_timer_;
    
    // State variables
    std::string current_state_;
    std::string robot_status_;
    int demo_counter_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<QuadrupedController>());
    rclcpp::shutdown();
    return 0;
}
