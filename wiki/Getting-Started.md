# Getting Started

This guide will help you set up the Weefee quadruped robot project on your system, including both the ESP32 firmware and ROS2 control software.

## Prerequisites

Before you begin, ensure you have the following:

- Ubuntu 22.04 (for ROS2 Humble) or Ubuntu 24.04 (for ROS2 Jazzy)
- Git
- Python 3.8 or newer
- Basic knowledge of Linux command line
- An ESP32 development board
- The quadruped robot hardware or simulation environment

## Installation Steps

### 1. Clone the Repository

```bash
git clone --recurse-submodules https://github.com/yourusername/weefee_project.git
cd weefee_project
```

### 2. Set Up ESP-IDF Environment

ESP-IDF is the official development framework for ESP32 microcontrollers.

1. Install ESP-IDF prerequisites:

```bash
sudo apt-get install git wget flex bison gperf python3 python3-pip python3-setuptools cmake ninja-build ccache libffi-dev libssl-dev dfu-util libusb-1.0-0
```

2. Clone ESP-IDF repository:

```bash
mkdir -p $HOME/esp
cd $HOME/esp
git clone --recursive https://github.com/espressif/esp-idf.git
cd esp-idf
git checkout release/v4.4
```

3. Run the installation script:

```bash
./install.sh
```

4. Set up environment variables:

```bash
. $HOME/esp/esp-idf/export.sh
```

### 3. Set Up micro-ROS

micro-ROS is essential for enabling ROS2 communication with the ESP32 microcontroller.

1. Install dependencies:

```bash
sudo apt update
sudo apt install python3-pip python3-colcon-common-extensions
pip3 install -U empy pyros-genmsg
```

2. Make sure submodules are properly initialized:

```bash
cd ~/weefee_project
git submodule update --init --recursive
```

3. The micro-ROS client library is included as an ESP-IDF component in the `/espidf/weefee_esp32/components/` directory.

4. The micro-ROS agent (host-side) needs to be installed on your computer:

```bash
# For ROS2 Humble (Ubuntu 22.04)
sudo apt install ros-humble-micro-ros-agent

# For ROS2 Jazzy (Ubuntu 24.04)
sudo apt install ros-jazzy-micro-ros-agent
```

### 4. Install ROS2

The project supports both ROS2 Humble (Ubuntu 22.04) and ROS2 Jazzy Jalisco (Ubuntu 24.04).

#### Option 1: ROS2 Humble (Ubuntu 22.04)

1. Set up the ROS2 repository:

```bash
sudo apt update && sudo apt install curl gnupg lsb-release
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(source /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
```

2. Install ROS2 Humble:

```bash
sudo apt update
sudo apt install ros-humble-desktop
```

3. Install additional ROS2 dependencies:

```bash
sudo apt install ros-humble-osrf-testing-tools-cpp ros-humble-test-msgs
sudo apt install ros-humble-geometry-msgs ros-humble-visualization-msgs
```

4. Set up ROS2 environment:

```bash
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

#### Option 2: ROS2 Jazzy Jalisco (Ubuntu 24.04)

1. Ubuntu 24.04 comes with ROS2 Jazzy Jalisco available in the standard repositories:

```bash
sudo apt update
sudo apt install ros-jazzy-desktop
```

2. Install additional ROS2 dependencies:

```bash
sudo apt install ros-jazzy-osrf-testing-tools-cpp ros-jazzy-test-msgs
sudo apt install ros-jazzy-geometry-msgs ros-jazzy-visualization-msgs
```

3. Set up ROS2 environment:

```bash
echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

## Building the Project

### ESP32 Firmware

1. Navigate to the ESP-IDF folder:

```bash
cd ~/weefee_project/espidf/weefee_esp32
```

2. Configure the project:

```bash
idf.py set-target esp32
idf.py menuconfig
```

In the menuconfig interface:
- Navigate to "Component config" → "micro-ROS Settings"
- Configure the WiFi settings or Ethernet if needed
- Configure the micro-ROS agent settings (IP and port)
- Save and exit

3. Build and flash the firmware:

```bash
idf.py build
idf.py -p /dev/ttyUSB0 flash  # Change ttyUSB0 to your ESP32 port
```

### ROS2 Control Software

1. Navigate to the ROS2 workspace:

```bash
cd ~/weefee_project/ros2_ws
```

2. Build the workspace:

```bash
colcon build
```

3. Source the environment:

```bash
source install/setup.bash
```

## Running the Project

### 1. Start the micro-ROS Agent

The micro-ROS agent serves as a bridge between ROS 2 and the ESP32 microcontroller:

```bash
# Use appropriate ROS2 distribution
ros2 run micro_ros_agent micro_ros_agent udp4 --port 8888
```

You should see the agent start and wait for connections from the ESP32.

### 2. Power On the Robot

Power on your quadruped robot or ESP32 development board. The ESP32 will:

1. Connect to WiFi (if configured)
2. Connect to the micro-ROS agent
3. Initialize the servo motors
4. Wait for commands

### 3. Run the ROS2 Control Node

```bash
ros2 run weefee_node servo_commander
```

### 4. Visualize with RViz (Optional)

Start the visualization node:

```bash
ros2 run weefee_node quadruped_visualizer
```

Then launch RViz:

```bash
rviz2
```

In RViz:
1. Add a MarkerArray display
2. Set the topic to "robot_visualization"
3. Set the fixed frame to "base_link"

### 5. Verify Communication

You can check if the robot is properly connected by:

```bash
# List all topics
ros2 topic list

# Monitor status messages
ros2 topic echo /robot_status

# Send a test command
ros2 topic pub --once /robot_command std_msgs/msg/String "{data: 'stand'}"
```

## Next Steps

- Learn about [the hardware setup](Hardware-Setup.md)
- Explore [the ESP32 firmware](ESP32-Firmware.md)
- Understand [the ROS2 control system](ROS2-Control.md)
- Learn about [quadruped kinematics](Kinematics.md)