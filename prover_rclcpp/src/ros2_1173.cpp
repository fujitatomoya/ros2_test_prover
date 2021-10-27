#include "rclcpp/rclcpp.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions options;
  options.automatically_declare_parameters_from_overrides(true);

  std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("ParametersNode", options);

  rclcpp::spin(node);
  rclcpp::shutdown();
}