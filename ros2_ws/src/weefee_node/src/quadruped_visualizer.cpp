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
#include "visualization_msgs/msg/marker_array.hpp"
#include "quadruped_inverse_kinematics.h"
#include <cmath>
#include <vector>
#include <string>

using namespace std::chrono_literals;

/**
 * @brief QuadrupedVisualizer - Provides ROS2 visualization of the quadruped robot
 * 
 * This node creates visual markers for the robot body, legs, and feet
 * and publishes them to be viewed in RViz. It also shows the robot's
 * current pose and movement state.
 */
class QuadrupedVisualizer : public rclcpp::Node
{
public:
    QuadrupedVisualizer()
    : Node("quadruped_visualizer")
    {
        // Publishers
        marker_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("robot_visualization", 10);
        
        // Subscribers
        cmd_sub_ = this->create_subscription<std_msgs::msg::String>(
            "robot_command", 10, std::bind(&QuadrupedVisualizer::command_callback, this, std::placeholders::_1));
        
        body_pose_sub_ = this->create_subscription<geometry_msgs::msg::Pose>(
            "body_pose", 10, std::bind(&QuadrupedVisualizer::body_pose_callback, this, std::placeholders::_1));
        
        servo_sub_ = this->create_subscription<std_msgs::msg::Int32MultiArray>(
            "servo_angles", 10, std::bind(&QuadrupedVisualizer::servo_callback, this, std::placeholders::_1));
        
        // Initialize kinematics engine
        kinematics_ = std::make_shared<QuadrupedInverseKinematics>();
        kinematics_->set_robot_dimensions(120.0f, 90.0f, 30.0f, 70.0f, 90.0f);
        
        // Visualization timer - update at 10Hz
        vis_timer_ = this->create_wall_timer(100ms, std::bind(&QuadrupedVisualizer::update_visualization, this));
        
        // Initialize robot state
        current_state_ = "INIT";
        body_position_ = {0.0f, 0.0f, 0.0f};
        body_orientation_ = {0.0f, 0.0f, 0.0f};
        
        RCLCPP_INFO(this->get_logger(), "Quadruped visualizer initialized");
    }

private:
    /**
     * @brief Process command messages to update state
     */
    void command_callback(const std_msgs::msg::String::SharedPtr msg)
    {
        std::string command = msg->data;
        
        if (command == "stand") {
            current_state_ = "STANDING";
        }
        else if (command == "sit") {
            current_state_ = "SITTING";
        }
        else if (command.substr(0, 4) == "walk") {
            current_state_ = "WALKING";
            
            // Extract speed if provided
            if (command.length() > 5) {
                try {
                    walk_speed_ = std::stof(command.substr(5));
                } catch (...) {
                    walk_speed_ = 1.0f;
                }
            }
        }
        else if (command == "trot") {
            current_state_ = "TROTTING";
        }
        else if (command == "stop") {
            current_state_ = "STOPPED";
        }
        else if (command.substr(0, 8) == "position") {
            // Position command format: "position x y z"
            std::istringstream iss(command);
            std::string cmd;
            float x, y, z;
            
            if (iss >> cmd >> x >> y >> z) {
                body_position_.x = x;
                body_position_.y = y;
                body_position_.z = z;
            }
        }
        else if (command.substr(0, 11) == "orientation") {
            // Orientation command format: "orientation roll pitch yaw"
            std::istringstream iss(command);
            std::string cmd;
            float roll, pitch, yaw;
            
            if (iss >> cmd >> roll >> pitch >> yaw) {
                body_orientation_.roll = roll;
                body_orientation_.pitch = pitch;
                body_orientation_.yaw = yaw;
            }
        }
        
        RCLCPP_INFO(this->get_logger(), "State updated to: %s", current_state_.c_str());
    }
    
    /**
     * @brief Process body pose updates
     */
    void body_pose_callback(const geometry_msgs::msg::Pose::SharedPtr msg)
    {
        body_position_.x = msg->position.x;
        body_position_.y = msg->position.y;
        body_position_.z = msg->position.z;
        
        // In a complete implementation, we would convert quaternion to Euler angles here
    }
    
    /**
     * @brief Process servo angle updates
     */
    void servo_callback(const std_msgs::msg::Int32MultiArray::SharedPtr msg)
    {
        if (msg->data.size() == 12) { // 4 legs * 3 joints
            // Update leg angles based on servo values
            for (int leg = 0; leg < 4; leg++) {
                for (int joint = 0; joint < 3; joint++) {
                    int servo_idx = leg * 3 + joint;
                    Leg* leg_ptr = kinematics_->get_leg(leg);
                    if (leg_ptr) {
                        leg_ptr->angles[joint] = static_cast<float>(msg->data[servo_idx]);
                    }
                }
            }
            
            // Update feet positions using forward kinematics
            update_foot_positions();
        }
    }
    
    /**
     * @brief Update foot positions using forward kinematics
     */
    void update_foot_positions()
    {
        for (int leg = 0; leg < LEG_COUNT; leg++) {
            Leg* leg_ptr = kinematics_->get_leg(leg);
            if (leg_ptr) {
                Vec3 foot_pos;
                kinematics_->forward_kinematics(leg, leg_ptr->angles, foot_pos);
                leg_ptr->foot_position = foot_pos;
            }
        }
    }
    
    /**
     * @brief Creates and publishes visualization markers for the robot
     */
    void update_visualization()
    {
        auto marker_array = visualization_msgs::msg::MarkerArray();
        
        // Add robot body
        auto body_marker = create_marker(0, "robot_body", "base_link", 
                                      body_position_.x, body_position_.y, body_position_.z);
        body_marker.type = visualization_msgs::msg::Marker::CUBE;
        body_marker.scale.x = 120.0; // Length
        body_marker.scale.y = 90.0;  // Width
        body_marker.scale.z = 20.0;  // Height
        body_marker.color.r = 0.2;
        body_marker.color.g = 0.2;
        body_marker.color.b = 0.8;
        marker_array.markers.push_back(body_marker);
        
        // Add text marker for state
        auto text_marker = create_marker(1, "robot_state", "base_link", 
                                      body_position_.x, body_position_.y, body_position_.z + 50.0);
        text_marker.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
        text_marker.text = current_state_;
        text_marker.scale.z = 30.0; // Text height
        text_marker.color.r = 1.0;
        text_marker.color.g = 1.0;
        text_marker.color.b = 1.0;
        marker_array.markers.push_back(text_marker);
        
        // Add legs and feet
        int marker_id = 2;
        for (int leg = 0; leg < LEG_COUNT; leg++) {
            Leg* leg_ptr = kinematics_->get_leg(leg);
            if (leg_ptr) {
                // Leg visualization (simplified as a line from mounting to foot)
                auto leg_marker = create_marker(marker_id++, "leg_" + std::to_string(leg), "base_link",
                                           0, 0, 0); // Will use points instead of pose
                leg_marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
                leg_marker.scale.x = 5.0; // Line width
                
                // Colors for different legs
                if (leg == LEG_FRONT_RIGHT || leg == LEG_FRONT_LEFT) {
                    leg_marker.color.r = 0.8;
                    leg_marker.color.g = 0.2;
                } else {
                    leg_marker.color.r = 0.2;
                    leg_marker.color.g = 0.8;
                }
                leg_marker.color.b = 0.2;
                
                // Add points for mounting and foot
                geometry_msgs::msg::Point mount_point;
                mount_point.x = leg_ptr->mounting_position.x + body_position_.x;
                mount_point.y = leg_ptr->mounting_position.y + body_position_.y;
                mount_point.z = leg_ptr->mounting_position.z + body_position_.z;
                leg_marker.points.push_back(mount_point);
                
                geometry_msgs::msg::Point foot_point;
                foot_point.x = leg_ptr->foot_position.x;
                foot_point.y = leg_ptr->foot_position.y;
                foot_point.z = leg_ptr->foot_position.z;
                leg_marker.points.push_back(foot_point);
                
                marker_array.markers.push_back(leg_marker);
                
                // Foot visualization
                auto foot_marker = create_marker(marker_id++, "foot_" + std::to_string(leg), "base_link",
                                            leg_ptr->foot_position.x,
                                            leg_ptr->foot_position.y,
                                            leg_ptr->foot_position.z);
                foot_marker.type = visualization_msgs::msg::Marker::SPHERE;
                foot_marker.scale.x = 10.0;
                foot_marker.scale.y = 10.0;
                foot_marker.scale.z = 10.0;
                foot_marker.color.r = 0.8;
                foot_marker.color.g = 0.8;
                foot_marker.color.b = 0.2;
                marker_array.markers.push_back(foot_marker);
            }
        }
        
        // Publish marker array
        marker_pub_->publish(marker_array);
    }
    
    /**
     * @brief Helper function to create a new marker
     */
    visualization_msgs::msg::Marker create_marker(int id, const std::string& ns, 
                                              const std::string& frame_id,
                                              float x, float y, float z)
    {
        visualization_msgs::msg::Marker marker;
        marker.header.frame_id = frame_id;
        marker.header.stamp = this->now();
        marker.ns = ns;
        marker.id = id;
        marker.action = visualization_msgs::msg::Marker::ADD;
        
        marker.pose.position.x = x;
        marker.pose.position.y = y;
        marker.pose.position.z = z;
        marker.pose.orientation.w = 1.0;
        
        marker.color.a = 1.0; // Alpha
        
        return marker;
    }

    // Publishers
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
    
    // Subscribers
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr cmd_sub_;
    rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr body_pose_sub_;
    rclcpp::Subscription<std_msgs::msg::Int32MultiArray>::SharedPtr servo_sub_;
    
    // Timer
    rclcpp::TimerBase::SharedPtr vis_timer_;
    
    // Robot state
    std::string current_state_;
    Vec3 body_position_;
    struct { float roll; float pitch; float yaw; } body_orientation_;
    float walk_speed_ = 1.0f;
    
    // Kinematics engine
    std::shared_ptr<QuadrupedInverseKinematics> kinematics_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<QuadrupedVisualizer>());
    rclcpp::shutdown();
    return 0;
}