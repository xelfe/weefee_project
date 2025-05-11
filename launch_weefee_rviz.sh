#!/bin/bash
#
# launch_weefee_rviz.sh - Script to launch Weefee robot with RViz visualization
#
# This script launches the weefee quadruped controller, visualizer, and RViz
# for visualizing the robot.
#
# Usage:
#   ./launch_weefee_rviz.sh - Start all components with RViz
#

# ANSI color codes for prettier output
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
RED='\033[0;31m'
NC='\033[0m' # No Color

# Workspace path
WS_PATH="$HOME/weefee_project"
ROS2_WS="$WS_PATH/ros2_ws"

# Print banner
echo -e "${BLUE}=======================================${NC}"
echo -e "${BLUE}  Weefee Robot RViz Visualization     ${NC}"
echo -e "${BLUE}=======================================${NC}"

# Source ROS2 environment
echo -e "\n${YELLOW}Sourcing ROS2 environment...${NC}"
source /opt/ros/jazzy/setup.bash 
source "$ROS2_WS/install/setup.bash"

# Launch using the launch file
echo -e "\n${YELLOW}Launching Weefee with RViz visualization...${NC}"
ros2 launch weefee_node weefee_launch.py

echo -e "\n${GREEN}Launch complete.${NC}"
