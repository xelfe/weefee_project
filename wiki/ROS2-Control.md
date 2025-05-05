# ROS2 Control

This page documents the ROS2 control software for the Weefee quadruped robot.

## Architecture Overview

The ROS2 control system consists of several nodes that provide different levels of control for the quadruped robot. The system is organized as follows:

```
ros2_ws/src/weefee_node/
├── CMakeLists.txt
└── src/
    ├── servo_commander.cpp               # Basic controller with demo sequence
    ├── quadruped_kinematics_controller.cpp # Advanced gait controller
    ├── quadruped_inverse_kinematics.h    # Kinematics implementation
    └── quadruped_visualizer.cpp          # Visualization for RViz
```

## ROS2 Nodes

### 1. Basic Controller (servo_commander.cpp)

The basic controller provides a simple interface for controlling the quadruped robot. It includes:

- Direct servo angle control
- Basic command generation (stand, sit, walk, etc.)
- Demo sequence showcasing various robot movements
- Status tracking

#### Key Features:

- Startup initialization of all servos to neutral positions
- Command publication to the robot
- Status subscription to receive robot feedback
- Automatic demo sequence showing different robot capabilities

#### Usage:

```bash
ros2 run weefee_node servo_commander
```

### 2. Kinematics Controller (quadruped_kinematics_controller.cpp)

The kinematics controller provides advanced control of the robot through inverse kinematics. It handles:

- Calculating foot positions based on gait patterns
- Converting foot positions to joint angles
- Managing robot body position and orientation
- Implementing different gait types (walk, trot)

#### Key Features:

- Forward and inverse kinematics
- Walking gait with customizable parameters
- Trotting gait implementation
- Body position and orientation adjustments
- 50Hz control loop for smooth motion

#### Usage:

```bash
ros2 run weefee_node quadruped_kinematics_controller
```

### 3. Robot Visualizer (quadruped_visualizer.cpp)

The visualizer node creates a 3D representation of the robot for visualization in RViz. It provides:

- Visual markers for the robot body
- Visual markers for the robot legs and feet
- State information display
- Position monitoring

#### Key Features:

- Visualization of the robot's physical structure
- Real-time updates based on robot commands
- Marker array publication for RViz
- Command subscription to track robot state

#### Usage:

```bash
ros2 run weefee_node quadruped_visualizer
```

## Communication Topics

The ROS2 control system uses the following topics:

| Topic | Type | Description |
|-------|------|-------------|
| `/robot_command` | std_msgs/String | Commands sent to the robot |
| `/robot_status` | std_msgs/String | Status messages from the robot |
| `/servo_angles` | std_msgs/Int32MultiArray | Direct servo angle control |
| `/body_pose` | geometry_msgs/Pose | Body position and orientation |
| `/gait_control` | std_msgs/String | Gait selection and control |
| `/robot_visualization` | visualization_msgs/MarkerArray | 3D visualization markers |

## Command Interface

Commands can be sent using ROS2 topics. The main command format uses string messages on the `/robot_command` topic:

| Command | Format | Description |
|---------|--------|-------------|
| Stand | `stand` | Put robot in standing position |
| Sit | `sit` | Put robot in sitting position |
| Walk | `walk [speed]` | Start walking gait |
| Trot | `trot` | Start trotting gait |
| Stop | `stop` | Stop movement |
| Position | `position x y z` | Set body position |
| Orientation | `orientation roll pitch yaw` | Set body orientation |

## Visualization in RViz

To visualize the robot in RViz:

1. Start the visualization node:
   ```bash
   ros2 run weefee_node quadruped_visualizer
   ```

2. Launch RViz:
   ```bash
   rviz2
   ```

3. Add a MarkerArray display:
   - Click "Add" → "MarkerArray"
   - Set the topic to "robot_visualization"
   - Set the fixed frame to "base_link"

RViz will show:
- Robot body (blue box)
- Legs (green/red lines)
- Feet (yellow spheres)
- Current state (text above the robot)

## Extending the System

The ROS2 control system is designed to be extensible. To add new functionality:

1. **New Commands**: Add new command parsing in the ESP32 firmware's `command_callback()`
2. **New Gaits**: Implement new gait algorithms in the kinematics controller
3. **New Visualizations**: Add new markers to the visualizer node

## Building

```bash
cd ros2_ws
colcon build --packages-select weefee_node
source install/setup.bash
```