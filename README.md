# 🐾 Weefee Project

A complete quadruped robot control system built on ESP32 (using ESP-IDF v5.4.1 + micro-ROS) and ROS 2 Jazzy, featuring inverse kinematics, multiple gait patterns, and ROS2 visualization.

![Quadruped Robot](https://i.imgur.com/example-placeholder.jpg)

## ✨ Key Features

- **Inverse & Forward Kinematics** - Precise leg position control
- **Multiple Gait Patterns** - Stand, walk, trot, and pace
- **Body Position & Orientation Control** - Full 6DOF body manipulation
- **Battery Monitoring System** - Real-time battery tracking with safety features
- **ROS2 Visualization** - 3D visualization in RViz

## 🚀 Quick Start

### Prerequisites

- ESP-IDF v5.4.1 (other versions are not guaranteed to work)
- ROS 2 Jazzy Jalisco (Ubuntu 24.04)
- micro-ROS for ESP-IDF v5.4.1

### 1. Clone the repository

```bash
git clone --recurse-submodules https://github.com/xelfe/weefee_project.git
cd weefee_project
```

### 2. Build & Flash ESP32 Firmware

```bash
cd espidf/weefee_esp32
. $HOME/esp/esp-idf/export.sh  # Make sure this points to ESP-IDF v5.4.1
idf.py build
idf.py -p /dev/ttyUSB0 flash
```

### 3. Build ROS2 Workspace

```bash
cd ../../ros2_ws
source /opt/ros/jazzy/setup.bash  # Specifically using ROS 2 Jazzy
colcon build
source install/setup.bash
```

### 4. Run the System

Terminal 1 - Start micro-ROS Agent:
```bash
source /opt/ros/jazzy/setup.bash
ros2 run micro_ros_agent micro_ros_agent udp4 --port 8888
```

Terminal 2 - Run Kinematics Controller:
```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 run weefee_node quadruped_kinematics_controller
```

Terminal 3 (Optional) - Run Visualizer with RViz:
```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 run weefee_node quadruped_visualizer
```

## 📚 Documentation

For detailed documentation, including installation instructions, code structure, and usage guides, please refer to our comprehensive wiki:

- [Project Wiki](https://github.com/xelfe/weefee_project/wiki)
- [Getting Started Guide](https://github.com/xelfe/weefee_project/wiki/Getting-Started)
- [Command Reference](https://github.com/xelfe/weefee_project/wiki/Command-Reference)
- [Kinematics Documentation](https://github.com/xelfe/weefee_project/wiki/Kinematics)
- [Battery Monitoring System](https://github.com/xelfe/weefee_project/wiki/Battery-Monitoring)

## 📖 Basic Commands

Control your robot with these basic ROS2 commands:

```bash
# Make the robot stand
ros2 topic pub --once /robot_command std_msgs/msg/String "{data: 'stand'}"

# Start walking at normal speed
ros2 topic pub --once /robot_command std_msgs/msg/String "{data: 'walk'}"

# Stop all movement
ros2 topic pub --once /robot_command std_msgs/msg/String "{data: 'stop'}"
```

## 🛠️ Project Structure

- **espidf/** – ESP32 firmware with micro-ROS integration
- **ros2_ws/** – ROS2 workspace with control nodes
- **wiki/** – Project documentation

## 📄 License

This project is licensed under the MIT License - see the [LICENSE](LICENSE) file for details.

## 👨‍💻 Development

This project is developed collaboratively with the assistance of AI (GitHub Copilot) to ensure efficient and robust solutions for quadruped robot control.

## 🔄 Last Updated

May 9, 2025