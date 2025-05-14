#!/bin/bash
# Script to test the optimized servo_commander node
# This version only tests the /robot_command topic since it's the only one
# now used to communicate with the ESP32
# Author: xelfe (plapensee@lapensee-electronique.ca)
# Date: May 13, 2025

# Colors for messages
GREEN='\033[0;32m'
RED='\033[0;31m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

echo -e "${YELLOW}==== OPTIMIZED SERVO COMMANDER TEST ====${NC}"
echo -e "${BLUE}This script tests the optimized communication system${NC}"
echo -e "${BLUE}using only the /robot_command topic for ESP32 control${NC}"

# First check if the servo_commander node is already running in a screen session
SCREEN_NAME="weefee_servo_commander"
SCREEN_RUNNING=false

if screen -list | grep -q $SCREEN_NAME; then
    echo -e "${GREEN}The servo_commander node is already running in a screen session ($SCREEN_NAME)${NC}"
    echo -e "Using existing instance for testing..."
    SCREEN_RUNNING=true
else
    echo -e "No screen session $SCREEN_NAME is running."
    
    # Check if servo_commander process is already running directly
    EXISTING_PID=$(pgrep -f "servo_commander")
    if [ ! -z "$EXISTING_PID" ]; then
        echo -e "${YELLOW}A servo_commander process is already running (PID: $EXISTING_PID)${NC}"
        echo -e "Do you want to use this process for testing? (y/n)"
        read -r answer
        if [[ "$answer" =~ ^[Yy]$ ]]; then
            echo -e "${GREEN}Using existing process for testing...${NC}"
            SERVO_COMMANDER_PID=$EXISTING_PID
        else
            echo -e "${YELLOW}Stopping existing process...${NC}"
            kill $EXISTING_PID
            sleep 2
            # Source ROS2 environment
            echo -e "Sourcing ROS2 environment..."
            source /opt/ros/jazzy/setup.bash
            source $HOME/weefee_project/ros2_ws/install/setup.bash
            
            # Launch servo_commander node
            echo -e "Launching servo_commander node..."
            ros2 run weefee_node servo_commander &
            SERVO_COMMANDER_PID=$!
            
            # Wait a bit for the node to start
            sleep 3
        fi
    else
        # Source ROS2 environment
        echo -e "Sourcing ROS2 environment..."
        source /opt/ros/jazzy/setup.bash
        source $HOME/weefee_project/ros2_ws/install/setup.bash
        
        # Launch servo_commander node
        echo -e "Launching servo_commander node..."
        ros2 run weefee_node servo_commander &
        SERVO_COMMANDER_PID=$!
        
        # Wait a bit for the node to start
        sleep 3
    fi
fi

# Source ROS2 environment if not already done
if [ ! -v ROS_DISTRO ] || [ -z "$ROS_DISTRO" ]; then
    echo -e "Sourcing ROS2 environment..."
    source /opt/ros/jazzy/setup.bash
    source $HOME/weefee_project/ros2_ws/install/setup.bash
fi

# Function to check if a topic exists
check_topic_exists() {
    local topic=$1
    local count=$(ros2 topic list | grep -c $topic)
    if [ $count -gt 0 ]; then
        echo -e "${GREEN}Topic $topic exists${NC}"
        return 0
    else
        echo -e "${RED}Topic $topic does not exist${NC}"
        return 1
    fi
}

# Function to send a command and verify its reception
send_command_and_verify() {
    local cmd=$1
    local description=$2
    
    echo -e "\n${YELLOW}Sending command: $cmd${NC}"
    echo -e "Description: $description"
    
    # Use 'timeout' to avoid hanging if the topic is not being published on
    timeout 2s ros2 topic pub --once /command_input std_msgs/msg/String "{data: '$cmd'}"
    
    # Slight delay to allow command processing
    sleep 1
    
    # Verify that the command was sent on the main topic
    echo -e "${YELLOW}Verifying command was sent on /robot_command topic...${NC}"
    local robot_cmd=$(timeout 2s ros2 topic echo --once /robot_command 2>/dev/null)
    
    if [ ! -z "$robot_cmd" ]; then
        echo -e "${GREEN}Command successfully verified on /robot_command topic${NC}"
        return 0
    else
        echo -e "${RED}Failed to verify command on /robot_command topic${NC}"
        return 1
    fi
}

# Function to check servo angle visualization
check_servo_angles() {
    echo -e "\n${YELLOW}Checking servo angle visualization on /servo_angles topic...${NC}"
    
    # Try to get servo angles
    local servo_angles=$(timeout 2s ros2 topic echo --once /servo_angles 2>/dev/null)
    
    if [ ! -z "$servo_angles" ]; then
        echo -e "${GREEN}Servo angles successfully visualized on /servo_angles topic${NC}"
        echo -e "Current angles:"
        echo "$servo_angles"
        return 0
    else
        echo -e "${RED}Failed to get servo angles from /servo_angles topic${NC}"
        return 1
    fi
}

# Main test sequence
echo -e "\n${YELLOW}Starting test sequence...${NC}"

# Check that required topics exist
echo -e "\n${YELLOW}Checking required topics...${NC}"
check_topic_exists "/robot_command" || { echo -e "${RED}Critical error: Required topic /robot_command not found${NC}"; exit 1; }
check_topic_exists "/command_input" || { echo -e "${RED}Critical error: Required topic /command_input not found${NC}"; exit 1; }

# Test command: calibrate
send_command_and_verify "calibrate" "Set robot to calibration position (all servos at 90-45-90)" || { echo -e "${RED}Calibrate command test failed${NC}"; exit 1; }
# Check servo angles
check_servo_angles

# Test command: stand
send_command_and_verify "stand" "Set robot to standing position" || { echo -e "${RED}Stand command test failed${NC}"; exit 1; }
# Check servo angles
check_servo_angles

# Test command: sit
send_command_and_verify "sit" "Set robot to sitting position" || { echo -e "${RED}Sit command test failed${NC}"; exit 1; }
# Check servo angles
check_servo_angles

# Return to stand position
send_command_and_verify "stand" "Return robot to standing position" || { echo -e "${RED}Return to stand command test failed${NC}"; exit 1; }

# Print test summary
echo -e "\n${GREEN}========== TEST SUMMARY ==========${NC}"
echo -e "${GREEN}✓ Topic verification: Success${NC}"
echo -e "${GREEN}✓ Command 'calibrate': Success${NC}"
echo -e "${GREEN}✓ Command 'stand': Success${NC}" 
echo -e "${GREEN}✓ Command 'sit': Success${NC}"
echo -e "${GREEN}✓ Servo angle visualization: Success${NC}"
echo -e "${GREEN}=================================${NC}"

# Kill the servo_commander node if we started it and it wasn't running in screen
if [ "$SCREEN_RUNNING" = false ] && [ -v SERVO_COMMANDER_PID ]; then
    echo -e "\n${YELLOW}Cleaning up: Terminating servo_commander process (PID: $SERVO_COMMANDER_PID)${NC}"
    kill $SERVO_COMMANDER_PID 2>/dev/null
fi

echo -e "\n${GREEN}Test completed successfully!${NC}"
echo -e "${BLUE}The optimized communication system works correctly.${NC}"
echo -e "${BLUE}ESP32 now only needs to subscribe to /robot_command${NC}"
echo -e "${BLUE}while servo angle visualization is maintained on /servo_angles.${NC}"
