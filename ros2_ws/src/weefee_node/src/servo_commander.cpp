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
#include <sstream>

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
        
        // Subscribers with custom QoS settings
        // Create a custom QoS profile with best effort reliability for micro-ROS compatibility
        auto qos = rclcpp::QoS(10).best_effort();
        
        status_sub_ = this->create_subscription<std_msgs::msg::String>(
            "robot_status", qos, std::bind(&QuadrupedController::status_callback, this, std::placeholders::_1));
        
        // Command subscriber - listen for manual commands
        command_input_sub_ = this->create_subscription<std_msgs::msg::String>(
            "command_input", 10, std::bind(&QuadrupedController::command_input_callback, this, std::placeholders::_1));
        
        // Initialize all servos to neutral position once at startup
        send_servo_positions({90, 90, 90, 90, 90, 90, 90, 90, 90, 90, 90, 90});
        
        // Note: Auto-demo timer is disabled to prevent automatic movements
        // ui_timer_ = this->create_wall_timer(1000ms, std::bind(&QuadrupedController::ui_update, this));
        
        RCLCPP_INFO(this->get_logger(), "Quadruped controller initialized in manual mode.");
        RCLCPP_INFO(this->get_logger(), "Send commands via the 'command_input' topic.");
        
        // Initialize state
        current_state_ = "IDLE";
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
     * @brief Callback for manual command input
     * Processes commands received via the command_input topic
     */
    void command_input_callback(const std_msgs::msg::String::SharedPtr msg)
    {
        const std::string& command = msg->data;
        RCLCPP_INFO(this->get_logger(), "Manual command received: %s", command.c_str());
        
        // Parse and process the command
        if (command == "stand") {
            stand_command();
            current_state_ = "STANDING";
        }
        else if (command == "sit") {
            sit_command();
            current_state_ = "SITTING";
        }
        else if (command == "stop") {
            stop_command();
            current_state_ = "STOPPED";
        }
        else if (command == "trot") {
            trot_command();
            current_state_ = "TROTTING";
        }
        else if (command.find("walk") == 0) {
            float speed = 1.0f;
            if (command.length() > 5) { // If there's something after "walk "
                try {
                    speed = std::stof(command.substr(5));  // Extract speed value
                } catch (const std::exception& e) {
                    RCLCPP_WARN(this->get_logger(), "Invalid walk speed format, using default speed 1.0");
                }
            }
            walk_command(speed);
            current_state_ = "WALKING";
        }
        else if (command.find("position") == 0) {
            std::istringstream iss(command);
            std::string cmd;
            float x, y, z;
            iss >> cmd >> x >> y >> z;
            
            if (iss.fail()) {
                RCLCPP_ERROR(this->get_logger(), "Invalid position command format. Use: position x y z");
            } else {
                set_body_position(x, y, z);
            }
        }
        else if (command.find("orientation") == 0) {
            std::istringstream iss(command);
            std::string cmd;
            float roll, pitch, yaw;
            iss >> cmd >> roll >> pitch >> yaw;
            
            if (iss.fail()) {
                RCLCPP_ERROR(this->get_logger(), "Invalid orientation command format. Use: orientation roll pitch yaw");
            } else {
                set_body_orientation(roll, pitch, yaw);
            }
        }
        else if (command.find("servo:") == 0) {
            // Direct servo command - forward as is
            auto servo_msg = std_msgs::msg::String();
            servo_msg.data = command;
            command_pub_->publish(servo_msg);
            RCLCPP_INFO(this->get_logger(), "Forwarded servo command to robot_command topic");
        }
        else {
            RCLCPP_WARN(this->get_logger(), "Unknown command: %s", command.c_str());
        }
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
     * @brief Sends servo positions to the robot via robot_command topic
     * @param positions Vector of servo positions
     */
    void send_servo_positions_via_command(const std::vector<int>& positions)
    {
        if (positions.size() != 12) {
            RCLCPP_ERROR(this->get_logger(), "Expected 12 servo positions, got %ld", positions.size());
            return;
        }
        
        // Format the servo command as: "servo:angle1,angle2,angle3,..."
        std::string cmd = "servo:";
        for (size_t i = 0; i < positions.size(); i++) {
            cmd += std::to_string(positions[i]);
            if (i < positions.size() - 1) {
                cmd += ",";
            }
        }
        
        auto msg = std_msgs::msg::String();
        msg.data = cmd;
        command_pub_->publish(msg);
        RCLCPP_DEBUG(this->get_logger(), "Servo positions sent via command: %s", cmd.c_str());
    }
    
    /**
     * @brief Sends servo positions to the robot
     * @param positions Vector of servo positions
     */
    void send_servo_positions(const std::vector<int>& positions)
    {
        if (positions.size() != 12) {
            RCLCPP_ERROR(this->get_logger(), "Expected 12 servo positions, got %ld", positions.size());
            return;
        }
        
        // Try to send via Int32MultiArray (for compatibility with standard ROS2 systems)
        auto servo_msg = std_msgs::msg::Int32MultiArray();
        
        // Configure message layout - essential for compatibility with micro-ROS
        servo_msg.layout.dim.push_back(std_msgs::msg::MultiArrayDimension());
        servo_msg.layout.dim[0].label = "servos";
        servo_msg.layout.dim[0].size = positions.size();
        servo_msg.layout.dim[0].stride = positions.size();
        servo_msg.layout.data_offset = 0;
        
        // Copy data
        servo_msg.data = positions;
        
        // Publish the message
        servo_pub_->publish(servo_msg);
        
        // ALSO send via robot_command to ensure compatibility with micro-ROS on ESP32
        send_servo_positions_via_command(positions);
        
        RCLCPP_DEBUG(this->get_logger(), "Servo positions published via both methods");
    }
    
    // Publishers
    rclcpp::Publisher<std_msgs::msg::Int32MultiArray>::SharedPtr servo_pub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr command_pub_;
    
    // Subscribers
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr status_sub_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr command_input_sub_;
    
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
