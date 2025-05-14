#!/bin/bash
# weefee.sh - Unified main script for the Weefee project
# Author: xelfe (plapensee@lapensee-electronique.ca)
# Date: May 13, 2025
#
# This unified script replaces multiple individual scripts and provides
# a consistent interface for all Weefee project operations.
#
# Usage:
#   ./weefee.sh launch           - Start all components
#   ./weefee.sh stop             - Stop all components
#   ./weefee.sh restart          - Restart all components
#   ./weefee.sh flash            - Compile and flash the ESP32
#   ./weefee.sh test             - Run all tests
#   ./weefee.sh test communication - Test communication
#   ./weefee.sh test visualization - Test visualization
#   ./weefee.sh demo             - Launch a feature demonstration
#   ./weefee.sh demo servo       - Demonstrate servo control
#   ./weefee.sh demo optimization - Compare optimizations
#   ./weefee.sh debug            - Debug mode
#   ./weefee.sh status           - Check system status
#   ./weefee.sh help             - Display this help message

# ANSI color codes
GREEN='\033[0;32m'
RED='\033[0;31m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

# Workspace path (adjust if needed)
WS_PATH="$HOME/weefee_project"
ROS2_WS="$WS_PATH/ros2_ws"
MICROROS_WS="$HOME/microros_ws"
ESP_PROJECT_PATH="$WS_PATH/espidf/weefee_esp32"

# Default ESP32 serial port - adjust if needed
ESP_PORT="/dev/ttyUSB0"

# Screen session names
AGENT_SCREEN="weefee_microros_agent"
NODE_SCREEN="weefee_controller"
VISUALIZER_SCREEN="weefee_visualizer"
KINEMATICS_SCREEN="weefee_kinematics"
SERVO_COMMANDER_SCREEN="weefee_servo_commander"

# Function to display headers
display_header() {
    echo -e "\n${BLUE}==========================================${NC}"
    echo -e "${BLUE}$1${NC}"
    echo -e "${BLUE}==========================================${NC}"
}

# Function to check if a topic exists
check_topic_exists() {
    local topic="$1"
    local count=$(ros2 topic list | grep -c "$topic")
    
    if [ "$count" -gt 0 ]; then
        echo -e "${GREEN}✓ Topic $topic exists${NC}"
        return 0
    else
        echo -e "${RED}✗ Topic $topic does not exist${NC}"
        return 1
    fi
}

# Source ROS2 environment
source_ros2() {
    source /opt/ros/jazzy/setup.bash
    source $ROS2_WS/install/setup.bash
}

# Launch all components
launch_components() {
    display_header "LAUNCHING WEEFEE COMPONENTS"
    
    # Check if screen is installed
    if ! command -v screen &> /dev/null; then
        echo -e "${YELLOW}Screen is not installed. Installing...${NC}"
        sudo apt update && sudo apt install -y screen
    fi
    
    # Stop any existing sessions first
    stop_components
    
    # Source ROS2 environment
    source_ros2
    
    echo -e "${YELLOW}Starting micro-ROS agent...${NC}"
    screen -dmS $AGENT_SCREEN bash -c "source $MICROROS_WS/install/setup.bash && ros2 run micro_ros_agent micro_ros_agent udp4 --port 8888 -v6; exec bash"
    sleep 2
    
    echo -e "${YELLOW}Starting servo controller...${NC}"
    screen -dmS $SERVO_COMMANDER_SCREEN bash -c "source $ROS2_WS/install/setup.bash && ros2 run weefee_node servo_commander; exec bash"
    sleep 1
    
    echo -e "${YELLOW}Starting visualizer...${NC}"
    screen -dmS $VISUALIZER_SCREEN bash -c "source $ROS2_WS/install/setup.bash && ros2 run weefee_node quadruped_visualizer; exec bash"
    sleep 1
    
    echo -e "${YELLOW}Starting kinematics controller...${NC}"
    screen -dmS $KINEMATICS_SCREEN bash -c "source $ROS2_WS/install/setup.bash && ros2 run weefee_node quadruped_kinematics_controller; exec bash"
    sleep 1
    
    echo -e "${GREEN}All components started successfully.${NC}"
    echo -e "To view micro-ROS agent logs: ${GREEN}screen -r $AGENT_SCREEN${NC}"
    echo -e "To view servo controller logs: ${GREEN}screen -r $SERVO_COMMANDER_SCREEN${NC}"
    echo -e "To view visualizer logs: ${GREEN}screen -r $VISUALIZER_SCREEN${NC}"
    echo -e "To view kinematics controller logs: ${GREEN}screen -r $KINEMATICS_SCREEN${NC}"
    echo -e "To detach from a screen session: Press ${GREEN}Ctrl+A${NC} followed by ${GREEN}D${NC}"
}

# Stop all components
stop_components() {
    display_header "STOPPING WEEFEE COMPONENTS"
    
    # In stop_components() function, add:
    if screen -list | grep -q "weefee_rviz"; then
        screen -S weefee_rviz -X quit
        echo -e "${GREEN}RViz screen session terminated.${NC}"
    fi
      
    # Check if the sessions exist and terminate them
    if screen -list | grep -q "$AGENT_SCREEN"; then
        screen -S $AGENT_SCREEN -X quit
        echo -e "${GREEN}Micro-ROS agent screen session terminated.${NC}"
    fi
    
    if screen -list | grep -q "$SERVO_COMMANDER_SCREEN"; then
        screen -S $SERVO_COMMANDER_SCREEN -X quit
        echo -e "${GREEN}Servo controller screen session terminated.${NC}"
    fi
    
    if screen -list | grep -q "$VISUALIZER_SCREEN"; then
        screen -S $VISUALIZER_SCREEN -X quit
        echo -e "${GREEN}Visualizer screen session terminated.${NC}"
    fi
    
    if screen -list | grep -q "$KINEMATICS_SCREEN"; then
        screen -S $KINEMATICS_SCREEN -X quit
        echo -e "${GREEN}Kinematics controller screen session terminated.${NC}"
    fi
    
    if screen -list | grep -q "$NODE_SCREEN"; then
        screen -S $NODE_SCREEN -X quit
        echo -e "${GREEN}Node screen session terminated.${NC}"
    fi
    
    # Also kill any stray processes
    kill_process "micro_ros_agent"
    kill_process "servo_commander"
    kill_process "quadruped_visualizer"
    kill_process "quadruped_kinematics_controller"
    
    echo -e "${GREEN}All components have been stopped.${NC}"
}

# Function to kill processes
kill_process() {
    local process_name="$1"
    local pid=$(pgrep -f "$process_name")
    
    if [ ! -z "$pid" ]; then
        echo -e "Process $process_name detected (PID: $pid), terminating..."
        kill $pid 2>/dev/null
        sleep 1
        if ps -p $pid > /dev/null; then
            kill -9 $pid 2>/dev/null
        fi
    fi
}

# Launch RViz for visualization
launch_rviz() {
    display_header "LAUNCHING RVIZ VISUALIZATION"
    
    # Source ROS2 environment
    source_ros2
    
    echo -e "${YELLOW}Starting RViz with quadruped configuration...${NC}"
    screen -dmS weefee_rviz bash -c "source /opt/ros/jazzy/setup.bash && source $ROS2_WS/install/setup.bash && ros2 run rviz2 rviz2 -d $ROS2_WS/src/weefee_node/config/quadruped.rviz; exec bash"
    
    echo -e "${GREEN}RViz started successfully.${NC}"
    echo -e "To view RViz screen session: ${GREEN}screen -r weefee_rviz${NC}"
}

# Flash ESP32
flash_esp32() {
    display_header "FLASHING ESP32"
    
    # Check if ESP-IDF is available
    if ! command -v idf.py &> /dev/null; then
        # Try to find ESP-IDF
        echo -e "${YELLOW}Searching for ESP-IDF...${NC}"
        
        # Check standard locations
        potential_paths=(
            "$HOME/esp/esp-idf"
            "/opt/esp-idf"
            "$HOME/esp-idf"
        )
        
        for idf_path in "${potential_paths[@]}"; do
            if [ -f "$idf_path/export.sh" ]; then
                echo -e "${YELLOW}ESP-IDF found at $idf_path${NC}"
                source "$idf_path/export.sh"
                break
            fi
        done
        
        if ! command -v idf.py &> /dev/null; then
            echo -e "${RED}ESP-IDF is required for flashing. Aborting.${NC}"
            exit 1
        fi
    fi
    
    # Navigate to ESP32 project directory
    cd "$ESP_PROJECT_PATH"
    
    # Check if serial port exists
    if [ ! -e "$ESP_PORT" ]; then
        echo -e "${YELLOW}Default serial port $ESP_PORT not found.${NC}"
        echo -e "${YELLOW}Available serial ports:${NC}"
        ls -l /dev/ttyUSB* /dev/ttyACM* 2>/dev/null || echo "No USB serial devices found"
        
        echo -e "${YELLOW}Please enter the serial port to use:${NC}"
        read -r input_port
        if [ -e "$input_port" ]; then
            ESP_PORT=$input_port
        else
            echo -e "${RED}Invalid port: $input_port. Aborting.${NC}"
            exit 1
        fi
    fi
    
    # Build the project
    echo -e "${YELLOW}Building ESP32 firmware...${NC}"
    idf.py build
    
    if [ $? -ne 0 ]; then
        echo -e "${RED}Build failed. Please check the error messages.${NC}"
        exit 1
    fi
    
    # Flash the device
    echo -e "${YELLOW}Flashing ESP32 device on $ESP_PORT...${NC}"
    idf.py -p "$ESP_PORT" flash
    
    if [ $? -ne 0 ]; then
        echo -e "${RED}Flash failed. Please check the error messages.${NC}"
        exit 1
    fi
    
    echo -e "${GREEN}ESP32 flash successful!${NC}"
}

# Test servo communication
test_communication() {
    display_header "SERVO COMMUNICATION TEST"
    
    source_ros2
    
    echo -e "${YELLOW}Checking available topics...${NC}"
    ros2 topic list | sort
    
    echo -e "\n${YELLOW}Testing essential topics:${NC}"
    check_topic_exists "/robot_command"
    check_topic_exists "/servo_angles"
    check_topic_exists "/robot_status"
    
    echo -e "\n${YELLOW}Sending a test command...${NC}"
    ros2 topic pub --once /robot_command std_msgs/msg/String "{data: 'stand'}"
    
    echo -e "\n${YELLOW}Checking response on /robot_status:${NC}"
    ros2 topic echo /robot_status --once
    
    echo -e "\n${YELLOW}Checking servo angles on /servo_angles:${NC}"
    ros2 topic echo /servo_angles --once
}

# Test visualization
test_visualization() {
    display_header "VISUALIZATION TEST"
    
    source_ros2
    
    # Start visualization listener in background
    echo -e "${YELLOW}Starting background listener for /servo_angles topic...${NC}"
    gnome-terminal --title="Servo Angles Visualization" -- bash -c "source /opt/ros/jazzy/setup.bash && source $ROS2_WS/install/setup.bash && ros2 topic echo /servo_angles; read -p 'Press Enter to close'" &
    viz_pid=$!
    sleep 2
    
    # Test calibration command
    echo -e "${YELLOW}Sending 'calibrate' command to /robot_command...${NC}"
    echo -e "This should generate servo angles on /servo_angles"
    ros2 topic pub --once /robot_command std_msgs/msg/String "{data: 'calibrate'}"
    echo -e "${GREEN}Command sent. Check visualization terminal.${NC}"
    sleep 5
    
    # Test stand command
    echo -e "${YELLOW}Sending 'stand' command to /robot_command...${NC}"
    echo -e "This should update servo angles to standing position"
    ros2 topic pub --once /robot_command std_msgs/msg/String "{data: 'stand'}"
    echo -e "${GREEN}Command sent. Check visualization terminal.${NC}"
    sleep 5
    
    # Test position command
    echo -e "${YELLOW}Sending position command to /robot_command...${NC}"
    echo -e "This should move the robot and update servo angle visualization"
    ros2 topic pub --once /robot_command std_msgs/msg/String "{data: 'position 20 0 15'}"
    echo -e "${GREEN}Command sent. Check visualization terminal.${NC}"
    sleep 5
    
    echo -e "\n${GREEN}Visualization test complete.${NC}"
    echo -e "The visualization terminal will remain open for reference."
    echo -e "You can close it manually when finished."
}

# Demo servo control
demo_servo() {
    display_header "SERVO CONTROL DEMO"
    
    source_ros2
    
    # Start visualization listener
    echo -e "${YELLOW}Starting servo angle visualization...${NC}"
    gnome-terminal --title="Servo Angles" -- bash -c "source /opt/ros/jazzy/setup.bash && source $ROS2_WS/install/setup.bash && ros2 topic echo /servo_angles; read -p 'Press Enter to close'" &
    viz_pid=$!
    sleep 2
    
    echo -e "${YELLOW}Servo demonstration sequence started...${NC}"
    
    echo -e "1. Calibrating servos..."
    ros2 topic pub --once /robot_command std_msgs/msg/String "{data: 'calibrate'}"
    sleep 3
    
    echo -e "2. Standing position..."
    ros2 topic pub --once /robot_command std_msgs/msg/String "{data: 'stand'}"
    sleep 3
    
    echo -e "3. Sitting position..."
    ros2 topic pub --once /robot_command std_msgs/msg/String "{data: 'sit'}"
    sleep 3
    
    echo -e "4. Adjusting orientation (roll 10, pitch -5)..."
    ros2 topic pub --once /robot_command std_msgs/msg/String "{data: 'orientation 10 -5 0'}"
    sleep 3
    
    echo -e "5. Adjusting position (height +20mm)..."
    ros2 topic pub --once /robot_command std_msgs/msg/String "{data: 'position 0 0 20'}"
    sleep 3
    
    echo -e "6. Returning to standing position..."
    ros2 topic pub --once /robot_command std_msgs/msg/String "{data: 'stand'}"
    sleep 3
    
    echo -e "${GREEN}Demonstration complete.${NC}"
}

# Demo optimization comparison
demo_optimization() {
    display_header "OPTIMIZATION COMPARISON"
    
    cat << EOF
${BLUE}BEFORE/AFTER OPTIMIZATION COMPARISON${NC}

${YELLOW}Architecture before optimization:${NC}
- ESP32 subscribed to 3 topics: /robot_command, /servo_angles, /servo_angles_str
- micro-ROS executor with capacity of 4
- Duplication of functionality between topics
- Separation of commands and visualization

${GREEN}Architecture after optimization:${NC}
- ESP32 only subscribes to /robot_command
- micro-ROS executor with capacity of 3 (25% improvement)
- Unified communication through a single channel
- Visualization handled on ROS2 side, not on ESP32
- Clear separation between commands and diagnostics

${YELLOW}Main changes:${NC}
1. Removal of redundant topics (/servo_angles, /servo_angles_str)
2. Creation of a helper method for visualization
3. Publication of servo angles for visualization from ROS2
4. Simplification of ESP32 code
5. Improved visualization for all commands
6. Updated documentation to reflect the new architecture

${GREEN}Benefits:${NC}
- Reduced load on ESP32
- Simpler and more maintainable architecture
- Better separation of responsibilities
- Preservation of all functionalities
- Improved system responsiveness
EOF
}

# Check system status
check_status() {
    display_header "SYSTEM STATUS CHECK"
    
    source_ros2
    
    echo -e "${YELLOW}Checking active screen sessions:${NC}"
    screen -list
    
    echo -e "\n${YELLOW}Checking available ROS2 topics:${NC}"
    ros2 topic list | sort
    
    echo -e "\n${YELLOW}Checking active ROS2 nodes:${NC}"
    ros2 node list
    
    echo -e "\n${YELLOW}Checking robot status:${NC}"
    ros2 topic echo /robot_status --once 2>/dev/null || echo -e "${RED}No message available on /robot_status${NC}"
    
    echo -e "\n${YELLOW}Checking ESP32 availability:${NC}"
    if [ -e "$ESP_PORT" ]; then
        echo -e "${GREEN}ESP32 is connected at $ESP_PORT${NC}"
    else
        echo -e "${RED}ESP32 not detected on $ESP_PORT${NC}"
        echo -e "${YELLOW}Available serial ports:${NC}"
        ls -l /dev/ttyUSB* /dev/ttyACM* 2>/dev/null || echo "No USB serial devices found"
    fi
}

# Run specific test
run_test() {
    local test_type="$1"
    
    case "$test_type" in
        "communication")
            test_communication
            ;;
        "visualization")
            test_visualization
            ;;
        *)
            display_header "RUNNING TESTS"
            echo -e "${YELLOW}Running all tests...${NC}"
            test_communication
            echo -e "\n"
            test_visualization
            ;;
    esac
}

# Run specific demo
run_demo() {
    local demo_type="$1"
    
    case "$demo_type" in
        "servo")
            demo_servo
            ;;
        "optimization")
            demo_optimization
            ;;
        *)
            display_header "WEEFEE ROBOT DEMONSTRATION"
            echo -e "${YELLOW}Running complete demo...${NC}"
            demo_servo
            echo -e "\n"
            demo_optimization
            ;;
    esac
}

# Debug mode
debug_mode() {
    display_header "DEBUG MODE"
    
    source_ros2
    
    echo -e "${YELLOW}Starting interactive debug mode...${NC}"
    echo -e "This mode allows you to send commands and see responses in real-time."
    
    # Start multiple terminal windows for monitoring
    gnome-terminal --title="Robot Status" -- bash -c "source /opt/ros/jazzy/setup.bash && source $ROS2_WS/install/setup.bash && echo -e '${BLUE}Monitoring /robot_status...${NC}' && ros2 topic echo /robot_status; read -p 'Press Enter to close'" &
    
    gnome-terminal --title="Servo Angles" -- bash -c "source /opt/ros/jazzy/setup.bash && source $ROS2_WS/install/setup.bash && echo -e '${BLUE}Monitoring /servo_angles...${NC}' && ros2 topic echo /servo_angles; read -p 'Press Enter to close'" &
    
    sleep 2
    
    echo -e "${YELLOW}Enter commands to send (q to quit):${NC}"
    while true; do
        echo -e "${GREEN}>> ${NC}"
        read -r cmd
        
        if [ "$cmd" == "q" ] || [ "$cmd" == "quit" ] || [ "$cmd" == "exit" ]; then
            break
        fi
        
        echo -e "${BLUE}Sending command: ${cmd}${NC}"
        ros2 topic pub --once /robot_command std_msgs/msg/String "{data: '$cmd'}"
        echo -e "${YELLOW}Check monitoring windows for responses${NC}"
    done
    
    echo -e "${GREEN}Debug mode terminated.${NC}"
}

# Help message
show_help() {
    display_header "WEEFEE SCRIPT HELP"
    
    cat << EOF
Usage: ./weefee.sh <command> [options]

Available commands:
  launch           - Start all components
  stop             - Stop all components
  restart          - Restart all components
  flash            - Compile and flash the ESP32
  test             - Run all tests
  test communication - Test communication
  test visualization - Test visualization
  demo             - Launch a feature demonstration
  demo servo       - Demonstrate servo control
  demo optimization - Compare optimizations
  debug            - Debug mode
  status           - Check system status
  help             - Display this help message

Examples:
  ./weefee.sh launch       - Start the complete system
  ./weefee.sh demo servo   - Demonstrate servo control
  ./weefee.sh test         - Run all tests
EOF
}

# Main function
main() {
    local command="$1"
    local option="$2"
    
    case "$command" in
        "launch")
            launch_components
            ;;
        "rviz")
            launch_rviz
            ;;
        "stop")
            stop_components
            ;;
        "restart")
            stop_components
            sleep 2
            launch_components
            ;;
        "flash")
            flash_esp32
            ;;
        "test")
            run_test "$option"
            ;;
        "demo")
            run_demo "$option"
            ;;
        "debug")
            debug_mode
            ;;
        "status")
            check_status
            ;;
        "help" | "-h" | "--help")
            show_help
            ;;
        *)
            echo -e "${RED}Unknown command: $command${NC}"
            show_help
            exit 1
            ;;
    esac
}

# Execute main function with all arguments
main "$@"
