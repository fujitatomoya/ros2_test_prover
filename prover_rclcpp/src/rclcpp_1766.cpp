#include <chrono>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

static rclcpp::Subscription<std_msgs::msg::String>::SharedPtr s_test_sub;

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<rclcpp::Node>("my_node");

    s_test_sub = node->create_subscription<std_msgs::msg::String>(
        "my_topic",
        rclcpp::SensorDataQoS(),
        [](std_msgs::msg::String::ConstSharedPtr msg) { (void)msg; });

    rclcpp::shutdown();
    return 0;
}