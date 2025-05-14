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
        // Publishers - command_pub_ is the main channel for communicating with the ESP32
        command_pub_ = this->create_publisher<std_msgs::msg::String>("robot_command", 10);
        
        // Adding a diagnostic-only publisher (ESP32 doesn't subscribe to it)
        servo_angles_pub_ = this->create_publisher<std_msgs::msg::Int32MultiArray>("servo_angles", 10);
        
        // Create a custom QoS profile with best effort reliability for micro-ROS compatibility
        auto qos = rclcpp::QoS(10).best_effort();
        
        // Subscribers
        status_sub_ = this->create_subscription<std_msgs::msg::String>(
            "robot_status", qos, std::bind(&QuadrupedController::status_callback, this, std::placeholders::_1));
        
        command_input_sub_ = this->create_subscription<std_msgs::msg::String>(
            "command_input", 10, std::bind(&QuadrupedController::command_input_callback, this, std::placeholders::_1));
        
        // Initialize servos to neutral position at startup
        send_servo_positions({90, 90, 90, 90, 90, 90, 90, 90, 90, 90, 90, 90});
        
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
        else if (command == "calibrate") {
            calibration_command();
            current_state_ = "CALIBRATING";
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
     * @brief Sends a command to move the robot to calibration position
     * 
     * Sets all servos to the assembly and calibration position (90,45,90).
     * This is the reference position used for all servo adjustments.
     */
    void calibration_command()
    {
        // High-level command for logging
        auto msg = std_msgs::msg::String();
        msg.data = "calibrate";
        command_pub_->publish(msg);
        RCLCPP_INFO(this->get_logger(), "Command sent: Calibration (high-level command)");
        
        // Also send positions via our optimized method
        std::vector<int> calibration_positions = {90, 45, 90, 90, 45, 90, 90, 45, 90, 90, 45, 90};
        
        // This call handles both sending the servo command and publishing to servo_angles topic
        send_servo_positions(calibration_positions, true);
    }
    
    /**
     * @brief Sends a command to the robot to stand up
     * 
     * Sets the robot in a standing position with legs properly positioned
     * for stability and balance.
     */
    void stand_command()
    {
        // High-level command for logging
        auto msg = std_msgs::msg::String();
        msg.data = "stand";
        command_pub_->publish(msg);
        RCLCPP_INFO(this->get_logger(), "Command sent: Stand (high-level command)");
        
        // Right side: femurs at 45°, tibias at 135° (adjusted from calibration position)
        // Left side: femurs at 45°, tibias at 45° (adjusted from calibration position)
        // Both configurations result in tibias perpendicular to the ground
        std::vector<int> stand_positions = {90, 45, 135, 90, 45, 45, 90, 45, 135, 90, 45, 45};
        
        // This call handles both sending the servo command and publishing to servo_angles topic
        send_servo_positions(stand_positions, true);
    }
    
    /**
     * @brief Sends a command to the robot to sit down
     * 
     * Sets the robot in a sitting position with front legs in standing
     * position and rear legs folded under the body.
     */
    void sit_command()
    {
        // Front legs: same as stand position
        // Rear right femur: raised to 15° (up from calibration position)
        // Rear left femur: set to 75° (adjusted for opposite servo orientation)
        // Rear tibias: at 135° (parallel to the ground)
        
        // High-level command for logging
        auto msg = std_msgs::msg::String();
        msg.data = "sit";
        command_pub_->publish(msg);
        RCLCPP_INFO(this->get_logger(), "Command sent: Sit (high-level command)");
        
        // Send positions using the optimized method which handles both
        // sending the servo command and publishing to servo_angles topic
        std::vector<int> sit_positions = {90, 45, 135, 90, 45, 45, 90, 15, 135, 90, 75, 135};
        send_servo_positions(sit_positions, true);
    }
    
    /**
     * @brief Sends a walking command with specified speed
     * @param speed Walking speed (default: 1.0)
     */
    void walk_command(float speed = 1.0f)
    {
        // High-level command for logging and control
        auto msg = std_msgs::msg::String();
        msg.data = "walk " + std::to_string(speed);
        command_pub_->publish(msg);
        RCLCPP_INFO(this->get_logger(), "Command sent: Walk at speed %.2f", speed);
        
        // Use the unified method to visualize walking position
        // This represents the starting position before the walking gait begins
        std::vector<int> walk_start_positions = {90, 45, 90, 90, 45, 90, 90, 45, 90, 90, 45, 90};
        
        // Only publish for visualization (false parameter means don't send commands to ESP32)
        publish_servo_angles_for_visualization(walk_start_positions);
    }
    
    /**
     * @brief Sends a trot gait command
     */
    void trot_command()
    {
        // High-level command for logging and control
        auto msg = std_msgs::msg::String();
        msg.data = "trot";
        command_pub_->publish(msg);
        RCLCPP_INFO(this->get_logger(), "Command sent: Trot");
        
        // Use the unified method to visualize trot position
        // This represents the starting position before the trot gait begins
        std::vector<int> trot_start_positions = {90, 35, 100, 90, 35, 100, 90, 35, 100, 90, 35, 100};
        
        // Only publish for visualization
        publish_servo_angles_for_visualization(trot_start_positions);
    }
    
    /**
     * @brief Sends command to stop any active movement
     */
    void stop_command()
    {
        // High-level command for logging and control
        auto msg = std_msgs::msg::String();
        msg.data = "stop";
        command_pub_->publish(msg);
        RCLCPP_INFO(this->get_logger(), "Command sent: Stop");
        
        // Use the unified method to visualize stop position
        // This represents the position where the robot stops in place
        std::vector<int> stop_positions = {90, 45, 90, 90, 45, 90, 90, 45, 90, 90, 45, 90};
        
        // Only publish for visualization
        publish_servo_angles_for_visualization(stop_positions);
    }
    
    /**
     * @brief Sets the body position of the robot
     * @param x X position (forward/backward)
     * @param y Y position (left/right)
     * @param z Z position (up/down)
     */
    void set_body_position(float x, float y, float z)
    {
        // High-level command for logging and control
        auto msg = std_msgs::msg::String();
        msg.data = "position " + std::to_string(x) + " " + 
                   std::to_string(y) + " " + std::to_string(z);
        command_pub_->publish(msg);
        RCLCPP_INFO(this->get_logger(), "Command sent: Position [%.2f, %.2f, %.2f]", x, y, z);
        
        // Visualize approximate servo angles for this position
        // Start with neutral position and adjust based on position parameters
        std::vector<int> position_angles = {90, 45, 90, 90, 45, 90, 90, 45, 90, 90, 45, 90};
        
        // Simple approximation for visualization (not actual inverse kinematics)
        // Adjust femur and tibia angles based on height (z)
        if (z > 10.0f) {
            // Higher position - extend legs
            for (int i = 1; i < 12; i += 3) { // femur angles
                position_angles[i] = 35;
            }
            for (int i = 2; i < 12; i += 3) { // tibia angles
                position_angles[i] = position_angles[i] + 15;
            }
        } else if (z < -10.0f) {
            // Lower position - compress legs
            for (int i = 1; i < 12; i += 3) { // femur angles
                position_angles[i] = 55;
            }
            for (int i = 2; i < 12; i += 3) { // tibia angles
                position_angles[i] = position_angles[i] - 10;
            }
        }
        
        // Adjust coxa angles for x/y position
        if (x > 10.0f) {
            // Forward position
            position_angles[0] = 80; // Front right
            position_angles[3] = 100; // Front left
            position_angles[6] = 100; // Rear right
            position_angles[9] = 80; // Rear left
        } else if (x < -10.0f) {
            // Backward position
            position_angles[0] = 100; // Front right
            position_angles[3] = 80; // Front left
            position_angles[6] = 80; // Rear right
            position_angles[9] = 100; // Rear left
        }
        
        // Y-axis position (left/right leaning)
        if (y > 10.0f) {
            // Right position - adjust leg heights
            position_angles[1] = position_angles[1] + 10; // Right legs higher
            position_angles[7] = position_angles[7] + 10;
            position_angles[4] = position_angles[4] - 10; // Left legs lower
            position_angles[10] = position_angles[10] - 10;
        } else if (y < -10.0f) {
            // Left position
            position_angles[1] = position_angles[1] - 10; // Right legs lower
            position_angles[7] = position_angles[7] - 10;
            position_angles[4] = position_angles[4] + 10; // Left legs higher
            position_angles[10] = position_angles[10] + 10;
        }
        
        // Only publish for visualization
        publish_servo_angles_for_visualization(position_angles);
    }
    
    /**
     * @brief Sets the body orientation of the robot
     * @param roll Roll angle (x-axis rotation)
     * @param pitch Pitch angle (y-axis rotation)
     * @param yaw Yaw angle (z-axis rotation)
     */
    void set_body_orientation(float roll, float pitch, float yaw)
    {
        // High-level command for logging and control
        auto msg = std_msgs::msg::String();
        msg.data = "orientation " + std::to_string(roll) + " " + 
                   std::to_string(pitch) + " " + std::to_string(yaw);
        command_pub_->publish(msg);
        RCLCPP_INFO(this->get_logger(), "Command sent: Orientation [%.2f, %.2f, %.2f]", 
                    roll, pitch, yaw);
                    
        // Visualize approximate servo angles for this orientation
        // Start with neutral position
        std::vector<int> orientation_angles = {90, 45, 90, 90, 45, 90, 90, 45, 90, 90, 45, 90};
        
        // Simple approximation for visualization (not actual inverse kinematics)
        // Roll affects the height of left vs right legs
        if (abs(roll) > 5.0f) {
            int roll_adjust = static_cast<int>(roll / 2.0f);
            // Right legs (0, 6)
            orientation_angles[1] = orientation_angles[1] - roll_adjust; // Front right femur
            orientation_angles[7] = orientation_angles[7] - roll_adjust; // Rear right femur
            
            // Left legs (3, 9)
            orientation_angles[4] = orientation_angles[4] + roll_adjust; // Front left femur
            orientation_angles[10] = orientation_angles[10] + roll_adjust; // Rear left femur
        }
        
        // Pitch affects the height of front vs rear legs
        if (abs(pitch) > 5.0f) {
            int pitch_adjust = static_cast<int>(pitch / 2.0f);
            // Front legs (0, 3)
            orientation_angles[1] = orientation_angles[1] - pitch_adjust; // Front right femur
            orientation_angles[4] = orientation_angles[4] - pitch_adjust; // Front left femur
            
            // Rear legs (6, 9)
            orientation_angles[7] = orientation_angles[7] + pitch_adjust; // Rear right femur
            orientation_angles[10] = orientation_angles[10] + pitch_adjust; // Rear left femur
        }
        
        // Yaw affects the angle of all coxa servos
        if (abs(yaw) > 5.0f) {
            int yaw_adjust = static_cast<int>(yaw / 3.0f);
            // Adjust all coxa servos (rotate the legs)
            orientation_angles[0] = orientation_angles[0] + yaw_adjust; // Front right coxa
            orientation_angles[3] = orientation_angles[3] - yaw_adjust; // Front left coxa
            orientation_angles[6] = orientation_angles[6] - yaw_adjust; // Rear right coxa
            orientation_angles[9] = orientation_angles[9] + yaw_adjust; // Rear left coxa
        }
        
        // Only publish for visualization
        publish_servo_angles_for_visualization(orientation_angles);
    }
    
    /**
     * @brief Helper method to publish servo angles for visualization only
     * @param positions Vector of servo positions
     */
    void publish_servo_angles_for_visualization(const std::vector<int>& positions)
    {
        if (positions.size() != 12) {
            RCLCPP_ERROR(this->get_logger(), "Expected 12 servo positions for visualization, got %ld", positions.size());
            return;
        }
        
        // Create and publish message
        auto array_msg = std_msgs::msg::Int32MultiArray();
        array_msg.data = positions;
        servo_angles_pub_->publish(array_msg);
        
        RCLCPP_DEBUG(this->get_logger(), "Published servo angles for visualization");
    }
    
    /**
     * @brief Sends servo positions to the robot and optionally publishes to servo_angles for visualization
     * @param positions Vector of servo positions
     * @param visualize Whether to publish to servo_angles topic for visualization (default: true)
     */
    void send_servo_positions(const std::vector<int>& positions, bool visualize = true)
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
        
        // Send the formatted command
        auto msg = std_msgs::msg::String();
        msg.data = cmd;
        command_pub_->publish(msg);
        
        // Use our helper method for visualization
        if (visualize) {
            publish_servo_angles_for_visualization(positions);
        }
        
        RCLCPP_INFO(this->get_logger(), "Servo positions sent: %s", cmd.c_str());
    }
    
    // Publishers
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr command_pub_;
    rclcpp::Publisher<std_msgs::msg::Int32MultiArray>::SharedPtr servo_angles_pub_;
    
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
