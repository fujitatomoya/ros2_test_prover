#include <iostream>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

void timer_callback(void)
{
    std::cerr << "timer ran\n";
}

void subscription_callback(std_msgs::msg::String::ConstSharedPtr msg)
{
    (void)msg;
}

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("rclcpp_create_timer");

    auto some_timer = rclcpp::create_timer(
        node,
        node->get_clock(),
        rclcpp::Duration(10, 0),
        timer_callback);

    // this does not bother us, since it handles as `AnySubscriptionCallback` object.
    auto sub = node->create_subscription<std_msgs::msg::String>(
        "my_topic",
        rclcpp::SensorDataQoS(),
        subscription_callback);

    rclcpp::executors::SingleThreadedExecutor exe;
    exe.add_node(node);
    exe.spin();

    rclcpp::shutdown();

    return 0;
}