#include <memory>
#include <rclcpp/rclcpp.hpp>

int main(void) {
  rclcpp::init(0, nullptr);
  {
    auto node = std::make_shared<rclcpp::Node>("my_node");
  }
  rclcpp::shutdown();
  return 0;
}
