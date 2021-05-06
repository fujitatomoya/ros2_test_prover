#include "rclcpp/rclcpp.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::Node::SharedPtr node = std::make_shared<rclcpp::Node>("my_node", "/ns");
  //rclcpp::Node::SharedPtr subnode = node->create_sub_node("sub_ns");
  rclcpp::Node::SharedPtr subnode = node->create_sub_node("");
  rclcpp::spin(node);
  rclcpp::shutdown();
}