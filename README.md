# 🐾 Weefee Project

This repository contains two main parts:

1. **espidf/** – Code for ESP32 using ESP-IDF + micro-ROS
2. **ros2_ws/** – Standard ROS 2 workspace with a C++ node to control the robot

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

#### For ROS 2
1. Install ROS 2 (e.g., Humble) by following the official instructions: [ROS 2 Installation Guide](https://docs.ros.org/en/humble/Installation.html).
2. Install additional required dependencies:
   ```bash
   sudo apt install ros-humble-osrf-testing-tools-cpp ros-humble-test-msgs
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
   idf.py flash
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
   source install/local_setup.bash
   ```

---

## 📚 Documentation

- [ESP-IDF Documentation](https://docs.espressif.com/projects/esp-idf/en/latest/esp32/index.html)
- [micro-ROS Documentation](https://micro.ros.org/)
- [ROS 2 Documentation](https://docs.ros.org/en/humble/index.html)
