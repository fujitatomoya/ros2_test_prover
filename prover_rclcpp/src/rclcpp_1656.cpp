#include "rclcpp/rclcpp.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::Node::SharedPtr node = std::make_shared<rclcpp::Node>("my_node", "/ns");
  RCLCPP_INFO(node->get_logger(),
    "Parent Node's effective name: %s", node->get_effective_namespace().c_str());
  rclcpp::Node::SharedPtr subnode = node->create_sub_node("sub_ns");
  //rclcpp::Node::SharedPtr subnode = node->create_sub_node(""); // generate exception here.
  RCLCPP_INFO(node->get_logger(),
    "Child Node's effective name: %s", subnode->get_effective_namespace().c_str());
  rclcpp::shutdown();
}