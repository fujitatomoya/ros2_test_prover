#include <rclcpp/rclcpp.hpp>

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);

  auto test_node = rclcpp::Node::make_shared("test_node", rclcpp::NodeOptions().arguments( {"--ros-args", "--disable-rosout-logs"}));
  //auto test_node = rclcpp::Node::make_shared("test_node");

  auto counter = 0;
  while (counter < 10) {
    counter++;
    RCLCPP_INFO(test_node->get_logger(), "Executor spining...");
    rclcpp::spin_some(test_node);
    std::this_thread::sleep_for(std::chrono::seconds(1));
  }
  rclcpp::shutdown();
  return 0;
}