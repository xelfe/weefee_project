# 🐾 Weefee Project

This repository contains two main parts:

1. **espidf/** – Code for ESP32 using ESP-IDF + micro-ROS
2. **ros2_ws/** – Standard ROS 2 workspace with C++ nodes to control the robot

## 🤖 Features

### Quadruped Robot Control
The project now includes a complete quadruped robot control system with:

- **Inverse & Forward Kinematics** - Calculate joint angles from foot positions and vice versa
- **Multiple Gait Patterns** - Stand, walk, trot, and pace movements
- **Body Position & Orientation Control** - Adjust the robot's stance and orientation
- **Servo Control System** - Direct control of all servomotors
- **ROS2 Control Interface** - Advanced control nodes with visualization support

---

## 👨‍💻 Development

This project is developed collaboratively with the assistance of AI (GitHub Copilot). The code architecture, algorithms, and implementation have been designed and optimized with AI guidance to ensure efficient and robust solutions for quadruped robot control.

---

## 🛠️ Installation

### Clone the repository
```bash
git clone --recurse-submodules https://github.com/<your_username>/weefee_project.git
cd weefee_project
```

### Install Dependencies

#### For ESP-IDF
1. Follow the official instructions to install ESP-IDF: [ESP-IDF Getting Started Guide](https://docs.espressif.com/projects/esp-idf/en/latest/esp32/get-started/index.html).
2. Set up the ESP-IDF environment:
   ```bash
   . $HOME/esp/esp-idf/export.sh
   ```

#### For micro-ROS
The project uses micro-ROS to enable ROS2 communication with the ESP32 microcontroller.

1. Install micro-ROS dependencies:
   ```bash
   sudo apt update
   sudo apt install python3-pip python3-colcon-common-extensions
   pip3 install -U empy pyros-genmsg
   ```
2. Ensure submodules are properly initialized:
   ```bash
   git submodule update --init --recursive
   ```
3. The micro-ROS client library is included as an ESP-IDF component in the `/espidf/weefee_esp32/components/` directory.

#### For ROS 2
The project supports both ROS2 Humble (Ubuntu 22.04) and ROS2 Jazzy Jalisco (Ubuntu 24.04):

##### ROS2 Humble (Ubuntu 22.04)
1. Install ROS 2 Humble:
   ```bash
   sudo apt install ros-humble-desktop
   sudo apt install ros-humble-osrf-testing-tools-cpp ros-humble-test-msgs
   sudo apt install ros-humble-geometry-msgs ros-humble-visualization-msgs
   ```

##### ROS2 Jazzy Jalisco (Ubuntu 24.04)
1. Install ROS 2 Jazzy:
   ```bash
   sudo apt install ros-jazzy-desktop
   sudo apt install ros-jazzy-osrf-testing-tools-cpp ros-jazzy-test-msgs
   sudo apt install ros-jazzy-geometry-msgs ros-jazzy-visualization-msgs
   ```

2. Set up ROS2 environment (for either version):
   ```bash
   # For Humble (Ubuntu 22.04)
   echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
   
   # For Jazzy (Ubuntu 24.04)
   echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc
   
   source ~/.bashrc
   ```

---

## 🚀 Build and Flash

### Build for ESP32
1. Navigate to the ESP-IDF folder:
   ```bash
   cd espidf/weefee_esp32
   ```
2. Configure the project:
   ```bash
   idf.py set-target esp32
   idf.py menuconfig
   ```
3. Build and flash the firmware:
   ```bash
   idf.py build
   idf.py -p /dev/ttyUSB0 flash
   ```

### Build for ROS 2
1. Navigate to the ROS 2 workspace:
   ```bash
   cd ros2_ws
   ```
2. Build the workspace:
   ```bash
   colcon build
   ```
3. Source the environment:
   ```bash
   source install/setup.bash
   ```

## 🖥️ Running the ROS2 Nodes

### 1. Start the micro-ROS Agent
First, start the micro-ROS agent to bridge between ROS2 and the ESP32:

```bash
# Install if you don't have it yet
sudo apt install ros-$ROS_DISTRO-micro-ros-agent

# Run the agent
ros2 run micro_ros_agent micro_ros_agent udp4 --port 8888
```

### 2. Run a Control Node
The ROS 2 workspace includes multiple nodes for different purposes:

1. **Basic controller**:
   ```bash
   ros2 run weefee_node servo_commander
   ```
   
2. **Kinematics controller** (advanced gait planning):
   ```bash
   ros2 run weefee_node quadruped_kinematics_controller
   ```
   
3. **Visualizer** (for use with RViz):
   ```bash
   ros2 run weefee_node quadruped_visualizer
   ```

To visualize the robot in RViz, add a MarkerArray display and subscribe to the "robot_visualization" topic.

---

## 📚 Documentation

- [Project Wiki](https://github.com/yourusername/weefee_project/wiki) - Full project documentation
- [ESP-IDF Documentation](https://docs.espressif.com/projects/esp-idf/en/latest/esp32/index.html)
- [micro-ROS Documentation](https://micro.ros.org/)
- [ROS 2 Humble Documentation](https://docs.ros.org/en/humble/index.html)
- [ROS 2 Jazzy Documentation](https://docs.ros.org/en/jazzy/index.html)

## 📋 Code Structure

### ESP32 Modules
- **quadruped_kinematics.h/c** - Mathematical functions for leg movement calculations
- **quadruped_robot.h/c** - High-level robot control and gait implementation 
- **servo_controller.h/c** - Low-level servo motor control interface
- **main.c** - Main application with micro-ROS integration

### ROS2 Modules
- **servo_commander.cpp** - Basic quadruped controller with demo sequence
- **quadruped_kinematics_controller.cpp** - Advanced controller with gait planning
- **quadruped_inverse_kinematics.h** - Inverse and forward kinematics implementation
- **quadruped_visualizer.cpp** - Robot visualization for RViz

## 🤖 Control Commands

The quadruped robot responds to the following commands:

- **stand**: Put the robot in standing position
- **sit**: Put the robot in sitting position
- **walk [speed]**: Start walking gait (optional speed parameter)
- **trot**: Start trotting gait
- **stop**: Stop any current movement
- **position x y z**: Set body position (in mm)
- **orientation roll pitch yaw**: Set body orientation (in degrees)

These commands can be sent via ROS2 topics or tested directly through the controller nodes.

The quadruped control system uses inverse kinematics to calculate joint angles from desired foot positions, supporting various gaits and body movements.