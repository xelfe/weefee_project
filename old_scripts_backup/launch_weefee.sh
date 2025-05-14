#!/bin/bash
#
# launch_weefee.sh - Script to launch Weefee robot components
#
# This script starts the micro-ROS agent and the ROS2 control node
# for the Weefee quadruped robot in separate screen sessions.
#
# Usage:
#   ./launch_weefee.sh         - Start all components
#   ./launch_weefee.sh stop    - Stop all components
#   ./launch_weefee.sh restart - Restart all components
#   ./launch_weefee.sh flash   - Compile and flash the ESP32     echo -e "To view micro-ROS agent logs: ${GREEN}screen -r $AGENT_SCREEN${NC}"
    echo -e "To view controller logs: ${GREEN}screen -r $NODE_SCREEN${NC}"
    echo -e "To view visualizer logs: ${GREEN}screen -r $VISUALIZER_SCREEN${NC}"
    echo -e "To view kinematics controller logs: ${GREEN}screen -r $KINEMATICS_SCREEN${NC}"
    echo -e "To view servo commander logs: ${GREEN}screen -r $SERVO_COMMANDER_SCREEN${NC}"
    echo -e "To detach from a screen session: Press ${GREEN}Ctrl+A${NC} followed by ${GREEN}D${NC}"are
#

# ANSI color codes for prettier output
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
RED='\033[0;31m'
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

# Print banner
echo -e "${BLUE}=======================================${NC}"
echo -e "${BLUE}     Weefee Robot Launch Script       ${NC}"
echo -e "${BLUE}=======================================${NC}"

# Check if screen is installed
if ! command -v screen &> /dev/null; then
    echo -e "${YELLOW}Screen is not installed. Installing...${NC}"
    sudo apt update && sudo apt install -y screen
    if [ $? -ne 0 ]; then
        echo -e "${RED}Failed to install screen. Please install it manually:${NC}"
        echo -e "sudo apt install -y screen"
        exit 1
    fi
    echo -e "${GREEN}Screen installed successfully.${NC}"
fi

# Function to check if micro_ros_agent is available
check_microros_agent() {
    if ! command -v micro_ros_agent &> /dev/null; then
        echo -e "${YELLOW}micro-ROS agent not found. Checking for installation...${NC}"
        
        # Check if microros workspace exists
        if [ -d "$MICROROS_WS" ]; then
            echo -e "${YELLOW}micro-ROS workspace found at $MICROROS_WS${NC}"
            
            # Check if the agent is built but not in PATH
            if [ -f "$MICROROS_WS/install/micro_ros_agent/lib/micro_ros_agent/micro_ros_agent" ]; then
                echo -e "${YELLOW}micro-ROS agent found in workspace. Adding to PATH...${NC}"
                export PATH=$PATH:$MICROROS_WS/install/micro_ros_agent/lib/micro_ros_agent
                return 0
            else
                echo -e "${YELLOW}micro-ROS agent not built in existing workspace.${NC}"
            fi
        else
            echo -e "${YELLOW}No micro-ROS workspace found.${NC}"
        fi
        
        echo -e "${YELLOW}Would you like to install micro-ROS agent? (y/n)${NC}"
        read -r answer
        if [[ "$answer" =~ ^[Yy]$ ]]; then
            echo -e "${YELLOW}Setting up micro-ROS agent...${NC}"
            
            # Source ROS2 environment first
            source /opt/ros/jazzy/setup.bash
            
            # Create workspace and clone micro_ros_setup
            mkdir -p $MICROROS_WS/src
            cd $MICROROS_WS/src
            git clone https://github.com/micro-ROS/micro_ros_setup.git
            cd $MICROROS_WS
            
            # Build micro_ros_setup
            colcon build
            source $MICROROS_WS/install/setup.bash
            
            # Create agent
            ros2 run micro_ros_setup create_agent_ws.sh
            ros2 run micro_ros_setup build_agent.sh
            
            # Check if installation was successful
            if [ -f "$MICROROS_WS/install/micro_ros_agent/lib/micro_ros_agent/micro_ros_agent" ]; then
                echo -e "${GREEN}micro-ROS agent installed successfully.${NC}"
                export PATH=$PATH:$MICROROS_WS/install/micro_ros_agent/lib/micro_ros_agent
                return 0
            else
                echo -e "${RED}Failed to install micro-ROS agent.${NC}"
                echo -e "${RED}Please follow the instructions at https://github.com/micro-ROS/micro_ros_setup${NC}"
                exit 1
            fi
        else
            echo -e "${RED}micro-ROS agent is required but installation was declined.${NC}"
            echo -e "${RED}Please install micro-ROS agent manually following these steps:${NC}"
            echo -e "${YELLOW}mkdir -p ~/microros_ws/src${NC}"
            echo -e "${YELLOW}cd ~/microros_ws/src${NC}"
            echo -e "${YELLOW}git clone https://github.com/micro-ROS/micro_ros_setup.git${NC}"
            echo -e "${YELLOW}cd ~/microros_ws${NC}"
            echo -e "${YELLOW}colcon build${NC}"
            echo -e "${YELLOW}source install/setup.bash${NC}"
            echo -e "${YELLOW}ros2 run micro_ros_setup create_agent_ws.sh${NC}"
            echo -e "${YELLOW}ros2 run micro_ros_setup build_agent.sh${NC}"
            exit 1
        fi
    fi
}

# Function to compile and flash ESP32 firmware
flash_esp32() {
    echo -e "\n${YELLOW}Compiling and flashing ESP32 firmware...${NC}"
    
    # Check if ESP-IDF is installed
    if ! command -v idf.py &> /dev/null; then
        echo -e "${RED}ESP-IDF tools not found in PATH.${NC}"
        echo -e "${YELLOW}Do you want to source ESP-IDF environment? (y/n)${NC}"
        read -r answer
        if [[ "$answer" =~ ^[Yy]$ ]]; then
            # Try common ESP-IDF locations
            if [ -f "$HOME/esp/esp-idf/export.sh" ]; then
                echo -e "${YELLOW}Sourcing ESP-IDF from $HOME/esp/esp-idf${NC}"
                source "$HOME/esp/esp-idf/export.sh"
            elif [ -f "/opt/esp-idf/export.sh" ]; then
                echo -e "${YELLOW}Sourcing ESP-IDF from /opt/esp-idf${NC}"
                source "/opt/esp-idf/export.sh"
            else
                echo -e "${RED}ESP-IDF not found in common locations.${NC}"
                echo -e "${YELLOW}Please enter the path to your ESP-IDF installation:${NC}"
                read -r idf_path
                if [ -f "$idf_path/export.sh" ]; then
                    echo -e "${YELLOW}Sourcing ESP-IDF from $idf_path${NC}"
                    source "$idf_path/export.sh"
                else
                    echo -e "${RED}Invalid ESP-IDF path or export.sh not found.${NC}"
                    echo -e "${RED}Please install ESP-IDF or add it to your PATH.${NC}"
                    echo -e "${RED}See https://docs.espressif.com/projects/esp-idf/en/latest/esp32/get-started/ for installation instructions.${NC}"
                    exit 1
                fi
            fi
        else
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
    
    echo -e "${GREEN}ESP32 firmware successfully built and flashed!${NC}"
    
    # Ask if user wants to monitor serial output
    echo -e "${YELLOW}Do you want to monitor the ESP32 serial output? (y/n)${NC}"
    read -r answer
    if [[ "$answer" =~ ^[Yy]$ ]]; then
        echo -e "${YELLOW}Starting serial monitor. Press Ctrl+C to exit.${NC}"
        idf.py -p "$ESP_PORT" monitor
    fi
}

# Function to stop all components
stop_components() {
    echo -e "\n${YELLOW}Stopping all Weefee components...${NC}"
    
    if screen -list | grep -q $AGENT_SCREEN; then
        echo -e "Terminating micro-ROS agent screen session..."
        screen -S $AGENT_SCREEN -X quit
        echo -e "${GREEN}Micro-ROS agent screen session terminated.${NC}"
    else
        echo -e "No micro-ROS agent screen session found."
    fi
    
    if screen -list | grep -q $NODE_SCREEN; then
        echo -e "Terminating controller screen session..."
        screen -S $NODE_SCREEN -X quit
        echo -e "${GREEN}Controller screen session terminated.${NC}"
    else
        echo -e "No controller screen session found."
    fi
    
    if screen -list | grep -q $VISUALIZER_SCREEN; then
        echo -e "Terminating visualizer screen session..."
        screen -S $VISUALIZER_SCREEN -X quit
        echo -e "${GREEN}Visualizer screen session terminated.${NC}"
    else
        echo -e "No visualizer screen session found."
    fi
    
    if screen -list | grep -q $KINEMATICS_SCREEN; then
        echo -e "Terminating kinematics controller screen session..."
        screen -S $KINEMATICS_SCREEN -X quit
        echo -e "${GREEN}Kinematics controller screen session terminated.${NC}"
    else
        echo -e "No kinematics controller screen session found."
    fi
    
    if screen -list | grep -q $SERVO_COMMANDER_SCREEN; then
        echo -e "Terminating servo commander screen session..."
        screen -S $SERVO_COMMANDER_SCREEN -X quit
        echo -e "${GREEN}Servo commander screen session terminated.${NC}"
    else
        echo -e "No servo commander screen session found."
    fi
    
    # Also kill any stray processes
    EXISTING_AGENT=$(pgrep -f "micro_ros_agent")
    if [ ! -z "$EXISTING_AGENT" ]; then
        echo -e "Found stray micro-ROS agent process (PID: $EXISTING_AGENT), terminating..."
        kill $EXISTING_AGENT 2>/dev/null
        sleep 1
        if ps -p $EXISTING_AGENT > /dev/null; then
            kill -9 $EXISTING_AGENT 2>/dev/null
        fi
    fi
    
    EXISTING_NODE=$(pgrep -f "quadruped_controller")
    if [ ! -z "$EXISTING_NODE" ]; then
        echo -e "Found stray quadruped controller process (PID: $EXISTING_NODE), terminating..."
        kill $EXISTING_NODE 2>/dev/null
        sleep 1
        if ps -p $EXISTING_NODE > /dev/null; then
            kill -9 $EXISTING_NODE 2>/dev/null
        fi
    fi
    
    EXISTING_VISUALIZER=$(pgrep -f "quadruped_visualizer")
    if [ ! -z "$EXISTING_VISUALIZER" ]; then
        echo -e "Found stray quadruped visualizer process (PID: $EXISTING_VISUALIZER), terminating..."
        kill $EXISTING_VISUALIZER 2>/dev/null
        sleep 1
        if ps -p $EXISTING_VISUALIZER > /dev/null; then
            kill -9 $EXISTING_VISUALIZER 2>/dev/null
        fi
    fi
    
    EXISTING_KINEMATICS=$(pgrep -f "quadruped_kinematics_controller")
    if [ ! -z "$EXISTING_KINEMATICS" ]; then
        echo -e "Found stray quadruped kinematics controller process (PID: $EXISTING_KINEMATICS), terminating..."
        kill $EXISTING_KINEMATICS 2>/dev/null
        sleep 1
        if ps -p $EXISTING_KINEMATICS > /dev/null; then
            kill -9 $EXISTING_KINEMATICS 2>/dev/null
        fi
    fi
    
    EXISTING_SERVO_COMMANDER=$(pgrep -f "servo_commander")
    if [ ! -z "$EXISTING_SERVO_COMMANDER" ]; then
        echo -e "Found stray servo commander process (PID: $EXISTING_SERVO_COMMANDER), terminating..."
        kill $EXISTING_SERVO_COMMANDER 2>/dev/null
        sleep 1
        if ps -p $EXISTING_SERVO_COMMANDER > /dev/null; then
            kill -9 $EXISTING_SERVO_COMMANDER 2>/dev/null
        fi
    fi
    
    echo -e "${GREEN}All components stopped.${NC}"
    sleep 2  # Wait for resources to be freed
}

# Function to start all components
start_components() {
    # Create a startup script for the micro-ROS agent
    AGENT_SCRIPT="/tmp/weefee_agent_start.sh"
    cat > $AGENT_SCRIPT << EOF
#!/bin/bash
source /opt/ros/jazzy/setup.bash
if [ -d "$MICROROS_WS" ]; then
    source $MICROROS_WS/install/setup.bash
fi
echo "Starting micro-ROS agent..."
micro_ros_agent udp4 --port 8888
EOF
    chmod +x $AGENT_SCRIPT

    # Create a startup script for the quadruped controller
    NODE_SCRIPT="/tmp/weefee_node_start.sh"
    cat > $NODE_SCRIPT << EOF
#!/bin/bash
source /opt/ros/jazzy/setup.bash
source $ROS2_WS/install/setup.bash
echo "Starting Weefee quadruped controller..."
sleep 3  # Wait for agent to initialize
ros2 run weefee_node quadruped_controller
EOF
    chmod +x $NODE_SCRIPT

    # Create a startup script for the quadruped visualizer
    VISUALIZER_SCRIPT="/tmp/weefee_visualizer_start.sh"
    cat > $VISUALIZER_SCRIPT << EOF
#!/bin/bash
source /opt/ros/jazzy/setup.bash
source $ROS2_WS/install/setup.bash
echo "Starting Weefee quadruped visualizer..."
sleep 3  # Wait for agent to initialize
ros2 run weefee_node quadruped_visualizer
EOF
    chmod +x $VISUALIZER_SCRIPT

    # Create a startup script for the quadruped kinematics controller
    KINEMATICS_SCRIPT="/tmp/weefee_kinematics_start.sh"
    cat > $KINEMATICS_SCRIPT << EOF
#!/bin/bash
source /opt/ros/jazzy/setup.bash
source $ROS2_WS/install/setup.bash
echo "Starting Weefee quadruped kinematics controller..."
sleep 3  # Wait for agent to initialize
ros2 run weefee_node quadruped_kinematics_controller
EOF
    chmod +x $KINEMATICS_SCRIPT

    # Create a startup script for the servo commander
    SERVO_COMMANDER_SCRIPT="/tmp/weefee_servo_commander_start.sh"
    cat > $SERVO_COMMANDER_SCRIPT << EOF
#!/bin/bash
source /opt/ros/jazzy/setup.bash
source $ROS2_WS/install/setup.bash
echo "Starting Weefee servo commander..."
sleep 3  # Wait for agent to initialize
echo "Running: ros2 run weefee_node servo_commander"
# Lister les topics avant de lancer le nœud
echo "Topics avant lancement:"
ros2 topic list

# Lancer le nœud avec plus de logs
ros2 run weefee_node servo_commander --ros-args --log-level debug

# Cette ligne ne sera jamais atteinte car ros2 run bloque, mais au cas où
echo "Après lancement du nœud servo_commander"
EOF
    chmod +x $SERVO_COMMANDER_SCRIPT

    # Launch micro-ROS agent in a screen session
    echo -e "\n${YELLOW}Starting micro-ROS agent in screen session '$AGENT_SCREEN'...${NC}"
    screen -dmS $AGENT_SCREEN $AGENT_SCRIPT

    # Check if agent screen session was created
    if ! screen -list | grep -q $AGENT_SCREEN; then
        echo -e "${RED}Failed to start micro-ROS agent screen session.${NC}"
        exit 1
    fi
    echo -e "${GREEN}micro-ROS agent started in screen session.${NC}"

    # Wait for the agent to initialize
    echo -e "Waiting for micro-ROS agent to initialize..."
    sleep 3

    # Launch ROS2 quadruped controller node in a screen session
    echo -e "\n${YELLOW}Starting Weefee quadruped controller in screen session '$NODE_SCREEN'...${NC}"
    screen -dmS $NODE_SCREEN $NODE_SCRIPT

    # Check if controller screen session was created
    if ! screen -list | grep -q $NODE_SCREEN; then
        echo -e "${RED}Failed to start Weefee controller screen session.${NC}"
        screen -S $AGENT_SCREEN -X quit
        exit 1
    fi
    echo -e "${GREEN}Weefee controller started in screen session.${NC}"
    
    # Launch ROS2 quadruped visualizer node in a screen session
    echo -e "\n${YELLOW}Starting Weefee quadruped visualizer in screen session '$VISUALIZER_SCREEN'...${NC}"
    screen -dmS $VISUALIZER_SCREEN $VISUALIZER_SCRIPT

    # Check if visualizer screen session was created
    if ! screen -list | grep -q $VISUALIZER_SCREEN; then
        echo -e "${RED}Failed to start Weefee visualizer screen session.${NC}"
        screen -S $AGENT_SCREEN -X quit
        screen -S $NODE_SCREEN -X quit
        exit 1
    fi
    echo -e "${GREEN}Weefee visualizer started in screen session.${NC}"
    
    # Launch ROS2 quadruped kinematics controller node in a screen session
    echo -e "\n${YELLOW}Starting Weefee quadruped kinematics controller in screen session '$KINEMATICS_SCREEN'...${NC}"
    screen -dmS $KINEMATICS_SCREEN $KINEMATICS_SCRIPT

    # Check if kinematics controller screen session was created
    if ! screen -list | grep -q $KINEMATICS_SCREEN; then
        echo -e "${RED}Failed to start Weefee kinematics controller screen session.${NC}"
        screen -S $AGENT_SCREEN -X quit
        screen -S $NODE_SCREEN -X quit
        screen -S $VISUALIZER_SCREEN -X quit
        exit 1
    fi
    echo -e "${GREEN}Weefee kinematics controller started in screen session.${NC}"
    
    # Launch ROS2 servo commander node in a screen session
    echo -e "\n${YELLOW}Starting Weefee servo commander in screen session '$SERVO_COMMANDER_SCREEN'...${NC}"
    screen -dmS $SERVO_COMMANDER_SCREEN $SERVO_COMMANDER_SCRIPT

    # Check if servo commander screen session was created
    if ! screen -list | grep -q $SERVO_COMMANDER_SCREEN; then
        echo -e "${RED}Failed to start Weefee servo commander screen session.${NC}"
        screen -S $AGENT_SCREEN -X quit
        screen -S $NODE_SCREEN -X quit
        screen -S $VISUALIZER_SCREEN -X quit
        screen -S $KINEMATICS_SCREEN -X quit
        exit 1
    fi
    echo -e "${GREEN}Weefee servo commander started in screen session.${NC}"

    echo -e "\n${GREEN}All components started successfully in screen sessions!${NC}"
    echo -e "${BLUE}=======================================${NC}"
    echo -e "${YELLOW}Weefee robot is now running${NC}"
    echo -e "${BLUE}=======================================${NC}"
    echo -e "\nTo view micro-ROS agent logs: ${GREEN}screen -r $AGENT_SCREEN${NC}"
    echo -e "To view controller logs: ${GREEN}screen -r $NODE_SCREEN${NC}"
    echo -e "To view visualizer logs: ${GREEN}screen -r $VISUALIZER_SCREEN${NC}"
    echo -e "To view kinematics controller logs: ${GREEN}screen -r $KINEMATICS_SCREEN${NC}"
    echo -e "To detach from a screen session: Press ${GREEN}Ctrl+A${NC} followed by ${GREEN}D${NC}"
    echo -e "To stop all components: ${GREEN}$0 stop${NC}"
    echo -e "To restart all components: ${GREEN}$0 restart${NC}"
}

# Check if pgrep is installed
if ! command -v pgrep &> /dev/null; then
    echo -e "${YELLOW}pgrep is not installed. Installing...${NC}"
    sudo apt update && sudo apt install -y procps
    if [ $? -ne 0 ]; then
        echo -e "${RED}Failed to install procps (pgrep). Please install it manually:${NC}"
        echo -e "sudo apt install -y procps"
        exit 1
    fi
    echo -e "${GREEN}pgrep installed successfully.${NC}"
fi

# Check if ROS2 is available
if ! command -v ros2 &> /dev/null; then
    echo -e "${RED}ROS2 not found. Please make sure ROS2 is installed and properly sourced.${NC}"
    exit 1
fi

# Handle command line arguments
if [ "$1" = "stop" ]; then
    stop_components
    exit 0
elif [ "$1" = "restart" ]; then
    echo -e "${YELLOW}Restarting all Weefee components...${NC}"
    stop_components
    # Continue with starting components
elif [ "$1" = "flash" ]; then
    # Flash ESP32 firmware
    flash_esp32
    exit 0
else
    # For start or no arguments, first stop any existing components
    stop_components
fi

# Check for micro-ROS agent
check_microros_agent

# Source micro-ROS workspace if it exists
if [ -d "$MICROROS_WS" ]; then
    echo -e "\n${YELLOW}Sourcing micro-ROS workspace...${NC}"
    source $MICROROS_WS/install/setup.bash
fi

# Source ROS2 environment
echo -e "\n${YELLOW}Sourcing ROS2 environment...${NC}"
source /opt/ros/jazzy/setup.bash 
# Vérifier si le fichier setup.bash existe avant de le sourcer
if [ -f "$ROS2_WS/install/setup.bash" ]; then
    source "$ROS2_WS/install/setup.bash"
else
    echo -e "${RED}Le fichier $ROS2_WS/install/setup.bash n'existe pas.${NC}"
    echo -e "${YELLOW}Il faut d'abord compiler le workspace ROS2 avec:${NC}"
    echo -e "${GREEN}cd $ROS2_WS && colcon build${NC}"
    exit 1
fi

# Start components
start_components