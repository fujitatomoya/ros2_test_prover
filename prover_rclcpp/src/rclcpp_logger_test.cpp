#include "rclcpp/rclcpp.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  rclcpp::Node::SharedPtr node_a = std::make_shared<rclcpp::Node>("node_a", "/ns");
  RCLCPP_INFO(node_a->get_logger(), "should be ns.node_a");
  auto child_logger_a = node_a->get_logger().get_child("child_logger_a");
  RCLCPP_INFO(child_logger_a, "should be ns.node_a.child_node_a");

  rclcpp::Node::SharedPtr node_b = std::make_shared<rclcpp::Node>("node_b", "/ns");
  RCLCPP_INFO(node_b->get_logger(), "should be ns.node_b");
  auto child_logger_b = node_b->get_logger().get_child("child_logger_b");
  RCLCPP_INFO(child_logger_b, "should be ns.node_b.child_logger_b");

  auto g_logger = rclcpp::get_logger("global_logger");
  RCLCPP_INFO(g_logger, "should be global_logger"); // does not go through /rosout but stdout.

  rclcpp::shutdown();
}