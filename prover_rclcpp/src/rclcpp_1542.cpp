#include <memory>
#include <rclcpp/rclcpp.hpp>

int main(void) {
  auto context = std::make_shared<rclcpp::Context>();
  context->init(0, nullptr);
  rclcpp::NodeOptions options;
  options.context(context);
  // Errors at this line
  auto node = std::make_shared<rclcpp::Node>("my_node", options);

  return 0;
}
