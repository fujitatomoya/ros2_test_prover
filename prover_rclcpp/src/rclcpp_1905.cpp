#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);

  rclcpp::NodeOptions node_options = rclcpp::NodeOptions();
  node_options.allow_undeclared_parameters(true);
  node_options.automatically_declare_parameters_from_overrides(true);

  auto node = rclcpp::Node::make_shared("testnode", node_options);
  RCLCPP_INFO(node->get_logger(), "/testnode created...");
  node->declare_parameter("bla", 0);
  RCLCPP_INFO(node->get_logger(), "declared parameter named bla...");

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  executor.spin();
  rclcpp::shutdown();

  return 0;
}