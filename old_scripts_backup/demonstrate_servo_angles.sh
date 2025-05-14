#!/bin/bash
# Script to demonstrate servo angle visualization
# Author: xelfe (plapensee@lapensee-electronique.ca)
# Date: May 13, 2025

# Colors for messages
GREEN='\033[0;32m'
RED='\033[0;31m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

echo -e "${YELLOW}Servo Angle Visualization Demonstration${NC}"
echo -e "${BLUE}This script demonstrates how to view servo angles when sending commands${NC}"

# Source ROS2 environment
source /opt/ros/jazzy/setup.bash
source $HOME/weefee_project/ros2_ws/install/setup.bash

# Function to check if a topic is available
check_topic() {
    local topic=$1
    local count=$(ros2 topic list | grep -c "$topic")
    if [ $count -gt 0 ]; then
        echo -e "${GREEN}Topic $topic is available${NC}"
        return 0
    else
        echo -e "${RED}Topic $topic is not available${NC}"
        return 1
    fi
}

# Check if the servo_commander node is running
SCREEN_NAME="weefee_servo_commander"

if ! screen -list | grep -q $SCREEN_NAME && ! pgrep -f "servo_commander" > /dev/null; then
    echo -e "${YELLOW}The servo_commander node is not running.${NC}"
    echo -e "Do you want to start it? (y/n)"
    read -r answer
    if [[ "$answer" =~ ^[Yy]$ ]]; then
        echo -e "${GREEN}Starting servo_commander node...${NC}"
        screen -dmS $SCREEN_NAME ros2 run weefee_node servo_commander
        sleep 3
    else
        echo -e "${RED}The servo_commander node is required for this test.${NC}"
        exit 1
    fi
fi

# Check required topics
echo -e "\n${YELLOW}Checking topics:${NC}"
check_topic "/robot_command" || exit 1
check_topic "/servo_angles" || exit 1

# Create a function to observe servo angles
observe_servo_angles() {
    echo -e "\n${YELLOW}Observing servo angles...${NC}"
    echo -e "Current angles are:"
    ros2 topic echo --once /servo_angles
}

# Function to send a command and observe angles
send_command_and_observe() {
    local command=$1
    local description=$2
    
    echo -e "\n${YELLOW}Sending command '$command'...${NC}"
    echo -e "${BLUE}$description${NC}"
    
    # Send the command
    ros2 topic pub --once /command_input std_msgs/msg/String "{data: '$command'}"
    
    # Wait a bit for the command to be processed
    sleep 1
    
    # Observe servo angles
    observe_servo_angles
}

# Demonstration of different commands
echo -e "\n${YELLOW}Servo angle visualization demonstration${NC}"

# Initial position
send_command_and_observe "calibrate" "Calibration position - servos at 90°, 45°, 90° for all legs"

# Standing position
send_command_and_observe "stand" "Standing position - legs are configured to support the robot"

# Sitting position
send_command_and_observe "sit" "Sitting position - front legs in standing position, rear legs folded"

echo -e "\n${GREEN}Demonstration completed!${NC}"
echo -e "${BLUE}This script shows how servo angles are published on /servo_angles${NC}"
echo -e "${BLUE}even though the ESP32 is not subscribed to this topic.${NC}"
echo -e "${BLUE}This is an optimization that reduces the load on the ESP32 while maintaining${NC}"
echo -e "${BLUE}visibility for diagnostics and debugging.${NC}"
