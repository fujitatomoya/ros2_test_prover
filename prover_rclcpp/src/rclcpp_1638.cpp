#include <rclcpp/rclcpp.hpp>
int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("foo");
  while(rclcpp::ok())
  {
    rclcpp::spin_some(node);
  }

  rclcpp::shutdown();
  return 0;
}