#include <rclcpp/rclcpp.hpp>
int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  // [original] auto node = std::make_shared<rclcpp::Node>("foo");
  // [no problem] auto executor = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
  // [no problem] executor->add_node(node);

  while(rclcpp::ok())
  {
    // [no problem] executor->spin();
    // [original] rclcpp::spin_some(node);
    auto executor = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
  }

  rclcpp::shutdown();
  return 0;
}