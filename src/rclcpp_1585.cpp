#include "rclcpp/rclcpp.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("vector_integer_parameters");

  //node->declare_parameter("param", std::vector<int>{});   // FAIL
  node->declare_parameter("param", std::vector<int64_t>{});   // SUCCESS

  rclcpp::spin(node);
  rclcpp::shutdown();
}