#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int32_multi_array.hpp"

using namespace std::chrono_literals;

class ServoCommander : public rclcpp::Node
{
public:
    ServoCommander()
    : Node("servo_commander")
    {
        publisher_ = this->create_publisher<std_msgs::msg::Int32MultiArray>("servo_angles", 10);
        timer_ = this->create_wall_timer(500ms, std::bind(&ServoCommander::publish_command, this));
        RCLCPP_INFO(this->get_logger(), "Servo commander lancé.");
    }

private:
    void publish_command()
    {
        auto message = std_msgs::msg::Int32MultiArray();
        message.data = {90, 45, 135, 90, 90, 90, 90, 90, 90, 90, 90, 90};  // Exemple : 12 servos
        publisher_->publish(message);
        RCLCPP_INFO(this->get_logger(), "Angles publiés.");
    }

    rclcpp::Publisher<std_msgs::msg::Int32MultiArray>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ServoCommander>());
    rclcpp::shutdown();
    return 0;
}
