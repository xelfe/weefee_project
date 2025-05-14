#!/bin/bash
# Test script for optimized servo commander architecture
# Demonstrates commands and visualization features

# Colors for output
GREEN='\033[0;32m'
BLUE='\033[0;34m'
YELLOW='\033[1;33m'
RED='\033[0;31m'
NC='\033[0m' # No Color

# Function to display section headers
display_header() {
    echo -e "\n${BLUE}==========================================${NC}"
    echo -e "${BLUE}$1${NC}"
    echo -e "${BLUE}==========================================${NC}"
}

# Source ROS2 environment
source /opt/ros/jazzy/setup.bash
source $HOME/weefee_project/ros2_ws/install/setup.bash

display_header "OPTIMIZATION TEST - UNIFIED COMMAND AND VISUALIZATION SYSTEM"
echo -e "This script demonstrates the optimized communication architecture."
echo -e "Commands are sent via a single topic (/robot_command), while"
echo -e "visualization data is automatically published to /servo_angles."

# Check ROS2 topics to verify our architecture
display_header "CHECKING AVAILABLE TOPICS"
echo -e "${YELLOW}Available topics should include /robot_command and /servo_angles${NC}"
ros2 topic list | grep -E "robot_command|servo_angles|robot_status"
echo -e "\n"

# Start visualization listener in background
display_header "STARTING SERVO ANGLE VISUALIZATION LISTENER"
echo -e "${YELLOW}Starting background listener for /servo_angles topic...${NC}"
gnome-terminal --title="Servo Angles Visualization" -- bash -c "source /opt/ros/jazzy/setup.bash && source $HOME/weefee_project/ros2_ws/install/setup.bash && ros2 topic echo /servo_angles; read -p 'Press Enter to close'" &
viz_pid=$!
sleep 2

# Test calibration command
display_header "TEST 1: CALIBRATION COMMAND"
echo -e "${YELLOW}Sending 'calibrate' command to /robot_command...${NC}"
echo -e "This should result in servo angles being published to /servo_angles"
echo -e "with all positions set to calibration values (90,45,90 pattern)"
ros2 topic pub --once /robot_command std_msgs/msg/String "{data: 'calibrate'}"
echo -e "${GREEN}Command sent. Check visualization terminal for servo angles.${NC}"
sleep 5

# Test stand command
display_header "TEST 2: STAND COMMAND"
echo -e "${YELLOW}Sending 'stand' command to /robot_command...${NC}"
echo -e "This should result in updated servo angles in the stand position"
ros2 topic pub --once /robot_command std_msgs/msg/String "{data: 'stand'}"
echo -e "${GREEN}Command sent. Check visualization terminal for servo angles.${NC}"
sleep 5

# Test sit command
display_header "TEST 3: SIT COMMAND"
echo -e "${YELLOW}Sending 'sit' command to /robot_command...${NC}"
echo -e "This should result in updated servo angles in the sit position"
ros2 topic pub --once /robot_command std_msgs/msg/String "{data: 'sit'}"
echo -e "${GREEN}Command sent. Check visualization terminal for servo angles.${NC}"
sleep 5

# Test direct servo control
display_header "TEST 4: DIRECT SERVO CONTROL"
echo -e "${YELLOW}Sending direct servo control command to /robot_command...${NC}"
echo -e "This should set specific angles and update the visualization"
ros2 topic pub --once /robot_command std_msgs/msg/String "{data: 'servo:90,60,120,90,60,60,90,60,120,90,60,60'}"
echo -e "${GREEN}Command sent. Check visualization terminal for servo angles.${NC}"
sleep 5

# Test position command
display_header "TEST 5: POSITION COMMAND WITH VISUALIZATION"
echo -e "${YELLOW}Sending position command to /robot_command...${NC}"
echo -e "This should move the robot and update servo angle visualization"
ros2 topic pub --once /robot_command std_msgs/msg/String "{data: 'position 20 0 15'}"
echo -e "${GREEN}Command sent. Check visualization terminal for servo angles.${NC}"
sleep 5

# Test orientation command
display_header "TEST 6: ORIENTATION COMMAND WITH VISUALIZATION"
echo -e "${YELLOW}Sending orientation command to /robot_command...${NC}"
echo -e "This should orient the robot and update servo angle visualization"
ros2 topic pub --once /robot_command std_msgs/msg/String "{data: 'orientation 10 -5 0'}"
echo -e "${GREEN}Command sent. Check visualization terminal for servo angles.${NC}"
sleep 5

# Show robot status
display_header "TEST 7: ROBOT STATUS MONITORING"
echo -e "${YELLOW}Checking robot status messages...${NC}"
echo -e "This shows status feedback from the robot:"
ros2 topic echo /robot_status --once

display_header "OPTIMIZATION TEST COMPLETE"
echo -e "${GREEN}All tests completed.${NC}"
echo -e "The visualization terminal will remain open for reference."
echo -e "You can close it manually when finished."

echo -e "\n${YELLOW}Key observations from this test:${NC}"
echo -e "1. All commands use a single topic (/robot_command)"
echo -e "2. Servo angle visualization works without the ESP32 having to subscribe to /servo_angles"
echo -e "3. The system provides effective separation between commands and diagnostics"
echo -e "4. Position and orientation commands now provide visualization approximations"
echo -e "5. The ESP32 experiences reduced load due to fewer subscriptions"
echo -e "6. All command methods use a consistent approach through helper methods"
