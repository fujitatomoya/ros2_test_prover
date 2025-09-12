#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);

  auto n = rclcpp::Node::make_shared("pub_node");
  auto pub = n->create_publisher<std_msgs::msg::String>("pub_topic", 10);

  std::thread turnOffPubThread([&n, &pub]() {
    std::this_thread::sleep_for(std::chrono::seconds(5));
    RCLCPP_INFO(n->get_logger(), "Shutting down publisher...");
    pub.reset(); // fastrtps and connextdds clears publisher endpoint from the network.
  });

  auto executor = rclcpp::executors::SingleThreadedExecutor();
  executor.add_node(n);

  auto counter = 0;
  while (counter < 10) {
    counter++;
    executor.spin_some();
    std::this_thread::sleep_for(std::chrono::seconds(1));
  }
  RCLCPP_INFO(n->get_logger(), "Shutting down rclcpp...");
  rclcpp::shutdown(); // cyclonedds, we can still see the publisher endpoint here.
  turnOffPubThread.join();
  return 0;
}