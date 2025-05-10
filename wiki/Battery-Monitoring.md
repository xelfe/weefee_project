# Battery Monitoring System

The Weefee robot includes a comprehensive battery monitoring system that tracks battery voltage, current consumption, and remaining capacity in real-time. This system helps prevent damage from over-discharge and provides timely status updates through ROS2.

## Overview

The battery monitoring system uses an INA219 current/voltage sensor connected to the ESP32 via I2C. It continuously monitors the battery status and publishes updates through the ROS2 status topic.

## Hardware Configuration

- **Sensor:** INA219 current/voltage sensor
- **Connection:** I2C interface (configurable pins)
- **Battery Support:** Designed for LiPo batteries (2S configuration, 7.4V nominal)
- **Measurement Capability:** Voltage, current, and power consumption

## Battery Status Levels

The system defines three battery status levels:

| Status | Description | Default Threshold (2S LiPo) |
|--------|-------------|----------------------------|
| **OK** | Normal operating voltage | Above 7.0V (>30% capacity) |
| **LOW** | Battery needs recharging soon | 7.0V to 6.6V (~10-30% capacity) |
| **CRITICAL** | Immediate recharge required | Below 6.6V (<10% capacity) |

## Safety Features

When the battery reaches **CRITICAL** level, the robot automatically:

1. Stops any current movement
2. Switches to a sitting position to prevent damage from falling
3. Continues sending status messages about the critical battery state

## Status Notifications

The battery status is regularly published on the `/robot_status` topic with messages in the following format:

```
Battery status: X.XXV (Y.Y%) - STATUS
```

Examples:
- `Battery status: 7.40V (85.0%) - OK`
- `Battery status: 7.10V (30.0%) - LOW`
- `Battery status: 6.80V (5.0%) - CRITICAL`

## Configuration Options

The battery monitoring system has several configurable parameters:

### ESP-IDF `menuconfig` Options

The following options can be configured through ESP-IDF's `menuconfig`:

```
Component config → Weefee Quadruped Configuration → Battery Monitoring
```

| Option | Description | Default |
|--------|-------------|---------|
| `BAT_MONITOR_ENABLED` | Enable/disable battery monitoring | Enabled |
| `BAT_MONITOR_SDA_PIN` | GPIO pin for I2C SDA | 21 |
| `BAT_MONITOR_SCL_PIN` | GPIO pin for I2C SCL | 22 |
| `BAT_MONITOR_I2C_FREQ` | I2C frequency in Hz | 100000 |
| `BAT_MONITOR_CELL_COUNT` | Number of battery cells | 2 |
| `BAT_MONITOR_UPDATE_INTERVAL` | Update interval in milliseconds | 5000 |

### Programmatic Configuration

The battery threshold levels can be adjusted programmatically:

```c
// Set custom threshold values (in volts)
esp_err_t ret = battery_monitor_set_thresholds(7.0f, 6.6f);
if (ret != ESP_OK) {
    ESP_LOGE(TAG, "Failed to set battery thresholds: %s", esp_err_to_name(ret));
}
```

## Reading Battery Status in ROS2

To read and react to battery status in your ROS2 application, subscribe to the `robot_status` topic:

### Python Example

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class BatteryMonitor(Node):
    def __init__(self):
        super().__init__('battery_monitor')
        self.status_subscription = self.create_subscription(
            String,
            'robot_status',
            self.status_callback,
            10)
        
    def status_callback(self, msg):
        status = msg.data
        if "Battery status" in status:
            # Extract voltage and percentage
            parts = status.split()
            voltage = float(parts[2].rstrip('V'))
            percentage = float(parts[3].strip('(').strip('%)'))
            state = parts[5]
            
            self.get_logger().info(f'Battery: {voltage}V ({percentage}%) - {state}')
            
            # React to battery states
            if state == "CRITICAL":
                self.get_logger().warn("CRITICAL BATTERY! Stopping all operations.")
                # Implement emergency actions here
            elif state == "LOW":
                self.get_logger().warn("Battery low, please recharge soon.")

def main():
    rclpy.init()
    battery_monitor = BatteryMonitor()
    rclpy.spin(battery_monitor)
    battery_monitor.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### C++ Example

```cpp
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include <string>
#include <regex>

class BatteryMonitor : public rclcpp::Node {
public:
  BatteryMonitor() : Node("battery_monitor") {
    status_subscription_ = this->create_subscription<std_msgs::msg::String>(
      "robot_status", 10, 
      std::bind(&BatteryMonitor::status_callback, this, std::placeholders::_1));
  }

private:
  void status_callback(const std_msgs::msg::String::SharedPtr msg) {
    std::string status = msg->data;
    if (status.find("Battery status") != std::string::npos) {
      // Extract battery information using regex
      std::regex pattern("Battery status: ([0-9.]+)V \\(([0-9.]+)%\\) - (OK|LOW|CRITICAL)");
      std::smatch matches;
      
      if (std::regex_search(status, matches, pattern) && matches.size() > 3) {
        float voltage = std::stof(matches[1].str());
        float percentage = std::stof(matches[2].str());
        std::string state = matches[3].str();
        
        RCLCPP_INFO(this->get_logger(), "Battery: %.2fV (%.1f%%) - %s", 
                   voltage, percentage, state.c_str());
        
        // React to battery states
        if (state == "CRITICAL") {
          RCLCPP_WARN(this->get_logger(), "CRITICAL BATTERY! Stopping all operations.");
          // Implement emergency actions here
        } else if (state == "LOW") {
          RCLCPP_WARN(this->get_logger(), "Battery low, please recharge soon.");
        }
      }
    }
  }
  
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr status_subscription_;
};

int main(int argc, char * argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<BatteryMonitor>());
  rclcpp::shutdown();
  return 0;
}
```

## Technical Details

### INA219 Sensor

The INA219 is a high-side current shunt and power monitor with an I2C interface. Key features:

- Senses bus voltages from 0 to 26V
- Reports current, voltage, and power
- High accuracy: 0.5% over temperature
- Low drift voltage reference
- Programmable calibration

### Battery Percentage Calculation

The remaining battery percentage is calculated based on a curve approximation that accounts for the non-linear discharge characteristics of LiPo batteries. For a 2S LiPo battery:

- 8.4V = 100% (fully charged)
- 7.4V = ~50%
- 6.6V = ~0% (protection cutoff)

## Troubleshooting

### Common Issues

1. **No battery readings**
   - Check I2C connection (SDA/SCL pins)
   - Verify I2C address (default 0x40)
   - Ensure battery monitor is enabled in configuration

2. **Incorrect voltage readings**
   - Verify correct cell count configuration
   - Check for proper calibration

3. **Battery percentage inconsistencies**
   - The percentage is an estimate and may not be perfectly accurate
   - Consider recalibrating if consistently off

### Diagnostic Commands

From the ESP-IDF monitor, you can use the following commands to diagnose issues:

```
# Check I2C communication
i2cdetect -y 0

# Read raw battery values
bat_read

# Set custom thresholds
bat_threshold 7.0 6.6
```

## Last Updated

May 9, 2025