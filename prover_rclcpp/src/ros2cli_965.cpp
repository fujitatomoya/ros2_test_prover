#include <rclcpp/rclcpp.hpp>

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);

  auto node = rclcpp::Node::make_shared("param_demo_cpp");

  rcl_interfaces::msg::ParameterDescriptor descriptor;
  descriptor.type = rclcpp::ParameterType::PARAMETER_STRING;

#if 0
  // this needs to be templated, so rclcpp does not allow user to set parameter without default nor type.
  //node->declare_parameter(  // cannot compile
  node->declare_parameter<std::string>( // rclcpp::exceptions::UninitializedStaticallyTypedParameterException
    "param_2_cpp",
    descriptor);
#else
  // this works as designed
  descriptor.dynamic_typing = true;
  descriptor.type = rclcpp::ParameterType::PARAMETER_NOT_SET;
  node->declare_parameter(
    "param_2_cpp",
    rclcpp::ParameterValue{},
    descriptor);
#endif

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  executor.spin();
  rclcpp::shutdown();

  return 0;
}