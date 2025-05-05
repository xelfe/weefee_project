# ESP32 Firmware

This page documents the ESP32-based firmware for the Weefee quadruped robot.

## Architecture Overview

The ESP32 firmware uses ESP-IDF (Espressif IoT Development Framework) and micro-ROS to control the quadruped robot hardware. The firmware is organized into several key components:

```
espidf/weefee_esp32/
├── main/
│   ├── main.c                 # Main application with micro-ROS integration
│   ├── quadruped_kinematics.h # Kinematics declarations
│   ├── quadruped_kinematics.c # Kinematics implementation
│   ├── quadruped_robot.h      # Robot control declarations
│   ├── quadruped_robot.c      # Robot control implementation
│   ├── servo_controller.h     # Servo control declarations
│   └── servo_controller.c     # Servo control implementation
└── components/
    └── micro_ros_espidf_component/ # micro-ROS integration
```

## Main Components

### 1. Servo Controller

The `servo_controller.h/c` files handle the low-level control of the servomotors. Key features:

- PWM signal generation using ESP32's LEDC peripheral
- Calibration and angle mapping
- Safety limits to prevent servo damage

Key functions:
- `setup_servos()` - Initialize all servos with default positions
- `set_servo_values()` - Update servo angle values
- `apply_servo_positions()` - Apply angles to physical servos

### 2. Quadruped Kinematics

The `quadruped_kinematics.h/c` files implement forward and inverse kinematics algorithms. These mathematical functions convert between:

- Joint angles (servo positions)
- Foot positions in 3D space

Key functions:
- `inverse_kinematics()` - Calculate joint angles from a desired foot position
- `forward_kinematics()` - Calculate foot position from joint angles
- `set_robot_dimensions()` - Configure the robot's physical dimensions

### 3. Quadruped Robot

The `quadruped_robot.h/c` files provide high-level robot control. This includes:

- Gaits (walking, trotting)
- Body position and orientation control
- Leg coordination

Key functions:
- `robot_init()` - Initialize the robot
- `robot_stand()` - Put the robot in standing position
- `robot_sit()` - Put the robot in sitting position
- `robot_start_gait()` - Begin a walking pattern
- `robot_stop_gait()` - Stop any active movement
- `robot_set_body_position()` - Adjust the body position
- `robot_set_body_orientation()` - Adjust the body orientation

### 4. Main Application

The `main.c` file is the entry point that integrates all components:

- Initializes the ESP32 hardware
- Sets up micro-ROS communication
- Processes commands from ROS2
- Controls the robot based on commands

The firmware uses micro-ROS to communicate with ROS2 over WiFi or Ethernet.

## Command Processing

The firmware uses a string-based command protocol. When commands are received via micro-ROS, they are parsed in the `command_callback()` function:

| Command Type | Format | Example | Description |
|-------------|--------|---------|-------------|
| Servo | `servo:angle1,angle2,...,angle12` | `servo:90,45,120,90,90,90,90,90,90,90,90,90` | Direct servo control |
| Stand | `stand` | `stand` | Put robot in standing position |
| Sit | `sit` | `sit` | Put robot in sitting position |
| Walk | `walk [speed]` | `walk 1.0` | Start walking gait |
| Trot | `trot` | `trot` | Start trotting gait |
| Stop | `stop` | `stop` | Stop movement |
| Position | `position x y z` | `position 0 0 20` | Set body position |
| Orientation | `orientation roll pitch yaw` | `orientation 0 10 0` | Set body orientation |

## Communication

The robot communicates using micro-ROS with the following topics:

| Topic | Type | Direction | Description |
|-------|------|-----------|-------------|
| `/robot_command` | std_msgs/String | Input | Commands to control the robot |
| `/robot_status` | std_msgs/String | Output | Status messages from the robot |
| `/robot_pose` | geometry_msgs/Pose | Input | Body position and orientation |

## Configuration

The ESP32 firmware can be configured using `idf.py menuconfig`. Important configuration options:

- WiFi credentials
- micro-ROS agent IP and port
- Servo pin assignments
- Robot dimensions

## Building and Flashing

```bash
cd espidf/weefee_esp32
idf.py build
idf.py -p /dev/ttyUSB0 flash
```

## Logs and Debugging

The firmware uses ESP-IDF's logging system. You can view logs with:

```bash
idf.py -p /dev/ttyUSB0 monitor
```

Log levels can be adjusted in `menuconfig` under "Component config" → "Log output".