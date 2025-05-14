#!/bin/bash
# Script to test servo angles communication in Weefee robot

# ANSI color codes
GREEN='\033[0;32m'
RED='\033[0;31m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

echo -e "${BLUE}==============================================${NC}"
echo -e "${BLUE}    Test de communication servo_angles        ${NC}"
echo -e "${BLUE}==============================================${NC}"

# Source ROS2 environment
echo -e "\n${YELLOW}Sourcing ROS2 environment...${NC}"
source /opt/ros/jazzy/setup.bash
source $HOME/weefee_project/ros2_ws/install/setup.bash

# Check topics
echo -e "\n${YELLOW}Checking for servo angles topics:${NC}"
ros2 topic list | grep servo_angles

# Check subscribers to servo_angles topics
echo -e "\n${YELLOW}Checking subscribers to servo_angles topics:${NC}"
echo -e "\nSubscribers to /servo_angles:"
ros2 topic info /servo_angles | grep "Subscription count"

echo -e "\nSubscribers to /servo_angles_str:"
ros2 topic info /servo_angles_str | grep "Subscription count"

echo -e "\n${YELLOW}Detailed topic information:${NC}"
echo -e "\nInfo for /servo_angles:"
ros2 topic info -v /servo_angles 2>/dev/null

echo -e "\nInfo for /servo_angles_str:"
ros2 topic info -v /servo_angles_str 2>/dev/null

# Check for ESP32 nodes
echo -e "\n${YELLOW}Checking for ESP32 nodes:${NC}"
ros2 node list | grep -i esp
ros2 node list | grep weefee

# Check micro-ROS agent status
echo -e "\n${YELLOW}Checking micro-ROS agent status:${NC}"
ps aux | grep micro_ros_agent

# Try publishing to servo_angles_str
echo -e "\n${YELLOW}Publishing test message to /servo_angles_str:${NC}"
ros2 topic pub --once /servo_angles_str std_msgs/msg/String "{data: '90,45,90,90,45,90,90,45,90,90,45,90'}"

# Try publishing to servo_angles with Int32MultiArray
echo -e "\n${YELLOW}Publishing test message to /servo_angles:${NC}"
ros2 topic pub --once /servo_angles std_msgs/msg/Int32MultiArray "{layout: {dim: [{label: 'servos', size: 12, stride: 12}], data_offset: 0}, data: [90, 45, 90, 90, 45, 90, 90, 45, 90, 90, 45, 90]}"

# Try sending a command through command_input
echo -e "\n${YELLOW}Sending 'calibrate' command through /command_input:${NC}"
ros2 topic pub --once /command_input std_msgs/msg/String "{data: 'calibrate'}"

# Directly access the screen session for the micro-ROS agent
echo -e "\n${YELLOW}Checking micro-ROS agent logs:${NC}"
screen -S weefee_microros_agent -X hardcopy /tmp/microros_agent.log
if [[ -f /tmp/microros_agent.log ]]; then
    cat /tmp/microros_agent.log
else
    echo -e "${RED}Could not retrieve micro-ROS agent logs${NC}"
fi

echo -e "\n${BLUE}==============================================${NC}"
echo -e "${YELLOW}Test de communication terminé${NC}"
echo -e "${BLUE}==============================================${NC}"
