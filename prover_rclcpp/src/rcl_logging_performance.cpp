#include <thread>
#include <vector>

#include <rclcpp/logger.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/rclcpp.hpp>

void log_debug(rclcpp::Node::SharedPtr node) {
    for (int i = 0; i < 1000000; i++) {
      RCLCPP_DEBUG(node->get_logger(), "Debug level logging");
    }
}

void log_info(rclcpp::Node::SharedPtr node) {
  for (int i = 0; i < 1000000; i++) {
    RCLCPP_INFO(node->get_logger(), "Information level logging");
  }
}

void log_warn(rclcpp::Node::SharedPtr node) {
  for (int i = 0; i < 1000000; i++) {
    RCLCPP_WARN(node->get_logger(), "Warning level logging");
  }
}

void log_error(rclcpp::Node::SharedPtr node) {
  for (int i = 0; i < 1000000; i++) {
    RCLCPP_ERROR(node->get_logger(), "Error level logging");
  }
}

void log_fatal(rclcpp::Node::SharedPtr node) {
  for (int i = 0; i < 1000000; i++) {
    RCLCPP_FATAL(node->get_logger(), "Fatal level logging");
  }
}

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::Node::SharedPtr node{rclcpp::Node::make_shared("node")};

  node->get_logger().set_level(rclcpp::Logger::Level::Debug);

  std::vector<std::thread> threads;
  threads.emplace_back(log_debug, node);
  threads.emplace_back(log_info, node);
  threads.emplace_back(log_warn, node);
  threads.emplace_back(log_error, node);
  threads.emplace_back(log_fatal, node);

  for (auto &thread : threads) {
    thread.join();
  }

  rclcpp::shutdown();

  return 0;
}
