# Command Reference

This page provides a complete reference for all commands supported by the Weefee quadruped robot.

## Command Interface

The quadruped robot accepts commands through the ROS2 topic `/robot_command` using the `std_msgs/String` message type. Commands can be sent using any ROS2 compatible method, including:

- Command line using `ros2 topic pub`
- Python scripts using the rclpy library
- C++ programs using the rclcpp library
- Custom applications using ROS2 client libraries

## Basic Movement Commands

| Command | Format | Example | Description |
|---------|--------|---------|-------------|
| Stand | `stand` | `stand` | Put the robot in a standing position with all feet on the ground |
| Sit | `sit` | `sit` | Lower the robot's body to a sitting position |
| Stop | `stop` | `stop` | Stop any current movement and maintain position |

## Gait Commands

| Command | Format | Example | Description |
|---------|--------|---------|-------------|
| Walk | `walk [speed]` | `walk` or `walk 1.5` | Start walking gait, with optional speed parameter (default 1.0) |
| Trot | `trot` | `trot` | Start trotting gait (diagonal legs move together) |

## Position & Orientation Commands

| Command | Format | Example | Description |
|---------|--------|---------|-------------|
| Position | `position x y z` | `position 0 0 20` | Set body position (in mm) relative to the default stance |
| Orientation | `orientation roll pitch yaw` | `orientation 0 10 0` | Set body orientation (in degrees) |

## Servo Control Commands

| Command | Format | Example | Description |
|---------|--------|---------|-------------|
| Servo | `servo:a1,a2,...,a12` | `servo:90,45,120,...` | Directly set all 12 servo angles (in degrees) |

## Command Line Examples

Here are examples of sending commands via the command line:

```bash
# Standing position
ros2 topic pub --once /robot_command std_msgs/msg/String "{data: 'stand'}"

# Start walking at 1.5x speed
ros2 topic pub --once /robot_command std_msgs/msg/String "{data: 'walk 1.5'}"

# Trotting gait
ros2 topic pub --once /robot_command std_msgs/msg/String "{data: 'trot'}"

# Stop movement
ros2 topic pub --once /robot_command std_msgs/msg/String "{data: 'stop'}"

# Set body position (20mm forward, 0mm sideways, 10mm up)
ros2 topic pub --once /robot_command std_msgs/msg/String "{data: 'position 20 0 10'}"

# Set body orientation (0 degrees roll, 10 degrees pitch, 5 degrees yaw)
ros2 topic pub --once /robot_command std_msgs/msg/String "{data: 'orientation 0 10 5'}"
```

## Python Example

Here's an example of controlling the robot using ROS2 Python:

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import time

class RobotController(Node):
    def __init__(self):
        super().__init__('robot_controller')
        self.command_publisher = self.create_publisher(String, 'robot_command', 10)
        self.status_subscription = self.create_subscription(
            String,
            'robot_status',
            self.status_callback,
            10)
        self.get_logger().info('Robot controller initialized')
        
    def send_command(self, command):
        msg = String()
        msg.data = command
        self.command_publisher.publish(msg)
        self.get_logger().info(f'Sent command: {command}')
        
    def status_callback(self, msg):
        self.get_logger().info(f'Robot status: {msg.data}')
        
        # Example of handling battery status messages
        if "Battery status" in msg.data:
            if "CRITICAL" in msg.data:
                self.get_logger().warn("Battery critically low, robot will sit to prevent damage")
            elif "LOW" in msg.data:
                self.get_logger().warn("Battery low, consider recharging soon")

def main():
    rclpy.init()
    controller = RobotController()
    
    # Demo sequence
    controller.send_command('stand')
    time.sleep(2)
    
    controller.send_command('position 20 0 0')
    time.sleep(2)
    
    controller.send_command('orientation 0 10 0')
    time.sleep(2)
    
    controller.send_command('walk 1.0')
    time.sleep(5)
    
    controller.send_command('stop')
    time.sleep(1)
    
    controller.send_command('sit')
    
    # Spin once to process any remaining callbacks
    rclpy.spin_once(controller, timeout_sec=1.0)
    
    # Clean up
    controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## C++ Example

Here's an example of controlling the robot using ROS2 C++:

```cpp
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include <chrono>
#include <functional>
#include <string>

using namespace std::chrono_literals;

class RobotController : public rclcpp::Node {
public:
  RobotController() : Node("robot_controller") {
    command_publisher_ = this->create_publisher<std_msgs::msg::String>("robot_command", 10);
    status_subscription_ = this->create_subscription<std_msgs::msg::String>(
      "robot_status", 10, 
      std::bind(&RobotController::status_callback, this, std::placeholders::_1));
      
    RCLCPP_INFO(this->get_logger(), "Robot controller initialized");
    
    // Start demo sequence after 1 second
    demo_timer_ = this->create_wall_timer(
      1000ms, std::bind(&RobotController::run_demo, this));
    
    demo_step_ = 0;
  }

private:
  void send_command(const std::string& command) {
    auto message = std_msgs::msg::String();
    message.data = command;
    command_publisher_->publish(message);
    RCLCPP_INFO(this->get_logger(), "Sent command: %s", command.c_str());
  }
  
  void status_callback(const std_msgs::msg::String::SharedPtr msg) {
    RCLCPP_INFO(this->get_logger(), "Robot status: %s", msg->data.c_str());
    
    // Example of handling battery status messages
    std::string status = msg->data;
    if (status.find("Battery status") != std::string::npos) {
      if (status.find("CRITICAL") != std::string::npos) {
        RCLCPP_WARN(this->get_logger(), "Battery critically low, robot will sit to prevent damage");
      } else if (status.find("LOW") != std::string::npos) {
        RCLCPP_WARN(this->get_logger(), "Battery low, consider recharging soon");
      }
    }
  }
  
  void run_demo() {
    switch (demo_step_) {
      case 0:
        send_command("stand");
        break;
      case 1:
        send_command("position 20 0 0");
        break;
      case 2:
        send_command("orientation 0 10 0");
        break;
      case 3:
        send_command("walk 1.0");
        break;
      case 8: // After 5 seconds of walking
        send_command("stop");
        break;
      case 9:
        send_command("sit");
        break;
      case 10:
        // End demo sequence
        demo_timer_->cancel();
        RCLCPP_INFO(this->get_logger(), "Demo complete");
        break;
    }
    
    demo_step_++;
  }
  
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr command_publisher_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr status_subscription_;
  rclcpp::TimerBase::SharedPtr demo_timer_;
  int demo_step_;
};

int main(int argc, char * argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<RobotController>());
  rclcpp::shutdown();
  return 0;
}
```

## Status Response

After sending commands, the robot responds with status messages on the `/robot_status` topic using the `std_msgs/String` message type. Example responses include:

| Command | Status Response |
|---------|----------------|
| `stand` | "Standing" |
| `sit` | "Sitting" |
| `walk 1.5` | "Walking at speed 1.50" |
| `trot` | "Trotting" |
| `stop` | "Stopped" |
| `position 20 0 10` | "Position set to [20.00, 0.00, 10.00]" |
| `orientation 0 10 5` | "Orientation set to [0.00, 10.00, 5.00]" |

## Battery Status

The robot periodically publishes battery status messages on the `/robot_status` topic. These messages include:

| Battery State | Status Message Format | Example |
|---------------|----------------------|---------|
| Normal | "Battery status: X.XXV (Y.Y%) - OK" | "Battery status: 7.40V (85.0%) - OK" |
| Low | "Battery status: X.XXV (Y.Y%) - LOW" | "Battery status: 7.10V (30.0%) - LOW" |
| Critical | "Battery status: X.XXV (Y.Y%) - CRITICAL" | "Battery status: 6.80V (5.0%) - CRITICAL" |

When the battery reaches a critical level, the robot will automatically:
1. Stop any current movement
2. Switch to a sitting position to prevent damage from falling
3. Continue sending status messages about the critical battery state

It is recommended to monitor these status messages in your application and implement appropriate handling for low battery conditions.

For more detailed information about the battery monitoring system, see the [Battery Monitoring](Battery-Monitoring.md) documentation.

## Error Handling

If the robot receives an invalid command, it will respond with an error message:

| Error Scenario | Status Response |
|----------------|----------------|
| Unknown command | "Unknown command: ..." |
| Invalid position parameters | "Error: position command requires 3 values, got ..." |
| Invalid orientation parameters | "Error: orientation command requires 3 values, got ..." |
| Invalid servo parameters | "Error: received X servo values, expected 12" |

## Additional Notes

- All position values are in millimeters
- All angle values are in degrees
- Servo angle ranges may be limited by mechanical constraints
- Position commands are relative to the default standing position
- Orientation commands are relative to the flat orientation (roll=0, pitch=0, yaw=0)
- Battery monitoring is performed automatically in the background
- The system uses a 2S LiPo battery (7.4V nominal)
- Low battery threshold is set at approximately 7.0V
- Critical battery threshold is set at approximately 6.6V

## Last Updated

May 9, 2025