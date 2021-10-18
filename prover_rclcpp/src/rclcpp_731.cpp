#include <chrono>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

static rclcpp::Subscription<std_msgs::msg::String>::SharedPtr s_test_sub;

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<rclcpp::Node>("my_node");
    auto sub1 = node->create_sub_node("sub1");
    auto sub2 = node->create_sub_node("sub2");

    sub1->declare_parameter("a", "foo");
    sub2->declare_parameter("b", "bar");
    rclcpp::spin(node);

    rclcpp::shutdown();
    return 0;
}