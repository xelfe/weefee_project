#!/bin/bash
# compare_optimizations.sh
# Script to demonstrate the optimizations made to the system
# Author: xelfe (plapensee@lapensee-electronique.ca)
# Date: May 13, 2025

# Colors for messages
GREEN='\033[0;32m'
RED='\033[0;31m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
CYAN='\033[0;36m'
MAGENTA='\033[0;35m'
NC='\033[0m' # No Color

# Source ROS2 environment
source /opt/ros/jazzy/setup.bash
source $HOME/weefee_project/ros2_ws/install/setup.bash

echo -e "${MAGENTA}============================================${NC}"
echo -e "${MAGENTA}  ESP32/ROS2 OPTIMIZATION DEMONSTRATION  ${NC}"
echo -e "${MAGENTA}============================================${NC}"

# Function to display a before/after comparison table
print_comparison_table() {
    echo -e "\n${CYAN}================================================${NC}"
    echo -e "${CYAN}           BEFORE/AFTER COMPARISON             ${NC}"
    echo -e "${CYAN}================================================${NC}"
    echo -e "${YELLOW}Aspect                  | Before     | After${NC}"
    echo -e "${CYAN}--------------------------|------------|------------${NC}"
    echo -e "${GREEN}Topics used              | 3          | 1${NC}"
    echo -e "${GREEN}Executor capacity        | 4          | 3${NC}"
    echo -e "${GREEN}Callbacks                | 2          | 1${NC}"
    echo -e "${GREEN}Structure size           | Larger     | Reduced${NC}"
    echo -e "${GREEN}Angle visualization      | From ESP32 | From ROS2${NC}"
    echo -e "${CYAN}================================================${NC}"
}

# Function to display microros_context_t structure optimizations
print_structure_optimization() {
    echo -e "\n${CYAN}================================================${NC}"
    echo -e "${CYAN}       MAIN STRUCTURE OPTIMIZATION             ${NC}"
    echo -e "${CYAN}================================================${NC}"
    
    echo -e "${YELLOW}Before:${NC}"
    echo -e "${BLUE}typedef struct {${NC}"
    echo -e "${BLUE}    rcl_node_t node;${NC}"
    echo -e "${BLUE}    rcl_subscription_t pose_sub;${NC}"
    echo -e "${BLUE}    rcl_subscription_t command_sub;${NC}"
    echo -e "${RED}    rcl_subscription_t servo_angles_sub;${NC}"
    echo -e "${BLUE}    rcl_publisher_t status_pub;${NC}"
    echo -e "${BLUE}    geometry_msgs__msg__Pose pose_msg;${NC}"
    echo -e "${BLUE}    std_msgs__msg__String command_msg;${NC}"
    echo -e "${RED}    std_msgs__msg__String servo_angles_msg;${NC}"
    echo -e "${BLUE}    std_msgs__msg__String status_msg;${NC}"
    echo -e "${BLUE}    rclc_executor_t executor;${NC}"
    echo -e "${BLUE}    rclc_support_t support;${NC}"
    echo -e "${BLUE}} microros_context_t;${NC}"
    
    echo -e "\n${YELLOW}After:${NC}"
    echo -e "${BLUE}typedef struct {${NC}"
    echo -e "${BLUE}    rcl_node_t node;${NC}"
    echo -e "${BLUE}    rcl_subscription_t pose_sub;${NC}"
    echo -e "${BLUE}    rcl_subscription_t command_sub;${NC}"
    echo -e "${BLUE}    rcl_publisher_t status_pub;${NC}"
    echo -e "${BLUE}    geometry_msgs__msg__Pose pose_msg;${NC}"
    echo -e "${BLUE}    std_msgs__msg__String command_msg;${NC}"
    echo -e "${BLUE}    std_msgs__msg__String status_msg;${NC}"
    echo -e "${BLUE}    rclc_executor_t executor;${NC}"
    echo -e "${BLUE}    rclc_support_t support;${NC}"
    echo -e "${BLUE}} microros_context_t;${NC}"
}

# Function to display ROS2-side modifications
print_ros2_modifications() {
    echo -e "\n${CYAN}================================================${NC}"
    echo -e "${CYAN}           ROS2-SIDE MODIFICATIONS             ${NC}"
    echo -e "${CYAN}================================================${NC}"
    
    echo -e "${YELLOW}Addition of a diagnostic publisher:${NC}"
    echo -e "${GREEN}servo_angles_pub_ = this->create_publisher<std_msgs::msg::Int32MultiArray>(\"servo_angles\", 10);${NC}"
    
    echo -e "\n${YELLOW}Command methods modification:${NC}"
    echo -e "${GREEN}// Before:${NC}"
    echo -e "${BLUE}void sit_command() {${NC}"
    echo -e "${BLUE}    // ... existing code ... ${NC}"
    echo -e "${BLUE}    std::vector<int> sit_positions = {90, 45, 135, 90, 45, 45, 90, 15, 135, 90, 75, 135};${NC}"
    echo -e "${BLUE}    send_servo_positions(sit_positions);${NC}"
    echo -e "${BLUE}}${NC}"
    
    echo -e "\n${GREEN}// After:${NC}"
    echo -e "${BLUE}void sit_command() {${NC}"
    echo -e "${BLUE}    // ... existing code ... ${NC}"
    echo -e "${BLUE}    std::vector<int> sit_positions = {90, 45, 135, 90, 45, 45, 90, 15, 135, 90, 75, 135};${NC}"
    echo -e "${GREEN}    // Publish directly to servo_angles for debugging${NC}"
    echo -e "${GREEN}    auto array_msg = std_msgs::msg::Int32MultiArray();${NC}"
    echo -e "${GREEN}    array_msg.data = sit_positions;${NC}"
    echo -e "${GREEN}    servo_angles_pub_->publish(array_msg);${NC}"
    echo -e "${BLUE}    send_servo_positions(sit_positions);${NC}"
    echo -e "${BLUE}}${NC}"
}

# Function to check available topics
check_available_topics() {
    echo -e "\n${CYAN}================================================${NC}"
    echo -e "${CYAN}             TOPIC VERIFICATION                ${NC}"
    echo -e "${CYAN}================================================${NC}"
    
    echo -e "${YELLOW}List of available topics:${NC}"
    ros2 topic list
    
    echo -e "\n${YELLOW}Checking main topic /robot_command:${NC}"
    if ros2 topic info /robot_command &>/dev/null; then
        echo -e "${GREEN}Topic /robot_command is available.${NC}"
    else
        echo -e "${RED}Topic /robot_command is not available!${NC}"
    fi
    
    echo -e "\n${YELLOW}Checking diagnostic topic /servo_angles:${NC}"
    if ros2 topic info /servo_angles &>/dev/null; then
        echo -e "${GREEN}Topic /servo_angles is available.${NC}"
    else
        echo -e "${RED}Topic /servo_angles is not available!${NC}"
    fi
}

# Main function
main() {
    echo -e "\n${YELLOW}This script demonstrates the optimizations made to the communication between ESP32 and ROS2${NC}"
    
    # Check if the servo_commander node is running
    if ! pgrep -f "servo_commander" > /dev/null; then
        echo -e "${RED}The servo_commander node is not running.${NC}"
        echo -e "${YELLOW}Please first run the launch_weefee.sh script${NC}"
        exit 1
    fi
    
    # Display the different parts of the demonstration
    print_comparison_table
    print_structure_optimization
    print_ros2_modifications
    check_available_topics
    
    # Conclusion
    echo -e "\n${CYAN}================================================${NC}"
    echo -e "${CYAN}                 CONCLUSION                    ${NC}"
    echo -e "${CYAN}================================================${NC}"
    echo -e "${GREEN}The optimizations we made have:${NC}"
    echo -e "${GREEN}1. Reduced the load on the ESP32 by eliminating unnecessary subscriptions${NC}"
    echo -e "${GREEN}2. Simplified the system architecture${NC}"
    echo -e "${GREEN}3. Maintained the servo angle visualization functionality${NC}"
    echo -e "${GREEN}4. Improved overall system performance${NC}"
    
    echo -e "\n${YELLOW}To see the servo angles in action, use:${NC}"
    echo -e "${BLUE}./demonstrate_servo_angles.sh${NC}"
}

# Execute the main script
main
