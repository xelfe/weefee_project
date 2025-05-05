# 🐾 Weefee Project Wiki

Welcome to the official wiki for the Weefee Quadruped Robot project! This wiki provides detailed documentation for both the ESP32 firmware and ROS2 control software for the quadruped robot.

## What is Weefee?

Weefee is a complete quadruped robot control system with advanced features including inverse/forward kinematics, multiple gait patterns, and full ROS2 integration. The project connects an ESP32 microcontroller to a ROS2 environment using micro-ROS, making it compatible with both ROS2 Humble (Ubuntu 22.04) and ROS2 Jazzy Jalisco (Ubuntu 24.04).

![Quadruped Robot Diagram](https://i.imgur.com/example-placeholder.jpg)

## Wiki Contents

* [Home](Home.md) - This page
* [Getting Started](Getting-Started.md) - Installation and initial setup
* [Hardware Setup](Hardware-Setup.md) - Hardware components and assembly
* [ESP32 Firmware](ESP32-Firmware.md) - Documentation for the ESP-IDF based firmware
* [ROS2 Control](ROS2-Control.md) - Documentation for the ROS2 control nodes
* [Kinematics](Kinematics.md) - Explanation of the inverse/forward kinematics implementation
* [Gait Patterns](Gait-Patterns.md) - Description of implemented walking gaits
* [Command Reference](Command-Reference.md) - Complete list of all robot commands
* [Visualization](Visualization.md) - Instructions for visualizing the robot in RViz
* [Troubleshooting](Troubleshooting.md) - Common issues and solutions
* [Contributing](Contributing.md) - Guidelines for contributing to the project

## Quick Links

* [Repository](https://github.com/yourusername/weefee_project)
* [Issue Tracker](https://github.com/yourusername/weefee_project/issues)
* [Latest Release](https://github.com/yourusername/weefee_project/releases/latest)

## Key Features

* **micro-ROS Integration** - Connects ESP32 to the ROS2 ecosystem
* **Multi-ROS2 Support** - Works with both ROS2 Humble and ROS2 Jazzy Jalisco
* **Inverse & Forward Kinematics** - Precise control of leg positions
* **Multiple Gait Patterns** - Walk, trot, and pace movements
* **Position & Orientation Control** - Adjust the robot's stance in 3D space
* **ROS2 Visualization** - 3D visualization in RViz

## System Architecture

```
┌─────────────────┐     ┌─────────────────┐     ┌─────────────────┐
│                 │     │                 │     │                 │
│   ROS2 Nodes    │◄────┤   micro-ROS     │◄────┤  ESP32 Firmware │
│  (Humble/Jazzy) │     │     Agent       │     │  (micro-ROS)    │
│                 │     │                 │     │                 │
└─────────────────┘     └─────────────────┘     └─────────────────┘
        ▲                                               │
        │                                               │
        └───────────────────────────────────────────────┘
                   Control & Feedback Loop
```

## License

This project is licensed under the MIT License - see the [LICENSE](https://github.com/yourusername/weefee_project/blob/main/LICENSE) file for details.