#include "rclcpp/rclcpp.hpp"

using namespace std::chrono_literals;

int main(int argc, const char* const* argv)
{
    rclcpp::init(argc, argv);
    rclcpp::Node::SharedPtr node{rclcpp::Node::make_shared("timer_test")};

    rclcpp::TimerBase::SharedPtr timer{node->create_timer(100ms, [node] {
        static rclcpp::Time prev{0, 0, RCL_ROS_TIME};
        const rclcpp::Time now{node->now()};
        RCLCPP_INFO(node->get_logger(), "time called: %ld", now.nanoseconds());
        RCLCPP_WARN_EXPRESSION(node->get_logger(), prev == now, "same timestamp: %ld", now.nanoseconds());
        prev = now;
    })};

    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
