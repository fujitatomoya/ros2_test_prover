#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);

  rclcpp::NodeOptions node_options = rclcpp::NodeOptions();
  node_options.allow_undeclared_parameters(true);
  node_options.automatically_declare_parameters_from_overrides(true);

  auto node1 = rclcpp::Node::make_shared("node1", "testns", node_options);
  RCLCPP_INFO(node1->get_logger(), "node1 created...");

  auto node2 = rclcpp::Node::make_shared("node2", "testns", node_options);
  RCLCPP_INFO(node2->get_logger(), "node2 created...");

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node1);
  executor.add_node(node2);
  executor.spin();
  rclcpp::shutdown();

  return 0;
}