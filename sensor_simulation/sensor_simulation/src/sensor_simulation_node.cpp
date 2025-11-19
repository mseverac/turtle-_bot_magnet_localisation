#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/u_int16.hpp>
#include <iostream>

using namespace std::chrono_literals;

class SensorSimulationNode : public rclcpp::Node
{
public:
    SensorSimulationNode()
    : Node("sensor_simulation_node")
    {
        publisher_ = this->create_publisher<std_msgs::msg::UInt16>("sensor_data", 10);

        // Set the timer to fire every 10 seconds
        timer_ = this->create_wall_timer(
            10s, std::bind(&SensorSimulationNode::timerCallback, this));
    }

private:
    void timerCallback()
    {
        std_msgs::msg::UInt16 msg;
        std::cout << "Enter a 16-bit unsigned integer: ";
        std::cin >> msg.data;

        publisher_->publish(msg);
        std::cout << "Published: " << msg.data << std::endl;
    }

    rclcpp::Publisher<std_msgs::msg::UInt16>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SensorSimulationNode>());
    rclcpp::shutdown();
    return 0;
}
