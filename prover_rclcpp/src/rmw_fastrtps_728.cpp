#include <iostream>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

void timer_callback(void)
{
  RCLCPP_INFO_ONCE(
    rclcpp::get_logger("rclcpp"), "timer_callback is called 1st time!");
}

int main(int argc, char ** argv)
{
  double pub_rate = 100000.0; // 100 kHz
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("timer_node");

  auto timer = rclcpp::create_timer(
    node,
    node->get_clock(),
    rclcpp::Duration(std::chrono::duration<double>(1.0 / pub_rate)),
    timer_callback);

  //rclcpp::executors::SingleThreadedExecutor exe;
  rclcpp::executors::MultiThreadedExecutor exe(rclcpp::ExecutorOptions(), 0, false, std::chrono::milliseconds(250));
  exe.add_node(node->get_node_base_interface());
  exe.spin();
  exe.remove_node(node->get_node_base_interface());

  rclcpp::shutdown();

  return 0;
}