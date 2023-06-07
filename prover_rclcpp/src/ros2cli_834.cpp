#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);

  rclcpp::NodeOptions node_options = rclcpp::NodeOptions();
  node_options.allow_undeclared_parameters(true);
  node_options.automatically_declare_parameters_from_overrides(true);

  auto node = rclcpp::Node::make_shared("param_test", node_options);
  RCLCPP_INFO(node->get_logger(), "/param_test node created...");
  // check if already declared
  rclcpp::Parameter param;
  if (node->has_parameter("test_double")) {
    param = node->get_parameter("test_double");
    RCLCPP_INFO(node->get_logger(), "test_double param is %f", param.get_value<double>());
  } else {
    // double type 0.0 initialized
    node->declare_parameter("test_double", 0.0);
  }
  // set parameter type double
  node->set_parameter(rclcpp::Parameter("test_double", 3e-06));
  param = node->get_parameter("test_double");
  RCLCPP_INFO(node->get_logger(), "test_double param is %f", param.get_value<double>());

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  executor.spin();
  rclcpp::shutdown();

  return 0;
}