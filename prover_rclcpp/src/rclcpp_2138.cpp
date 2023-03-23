#include <time.h>
#include <chrono>
#include <iostream>
#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/empty.hpp>

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("test_node");
  auto executor = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
  executor->add_node(node);
  auto fut = std::async(std::launch::async, [executor] { executor->spin(); });

  {
    RCLCPP_INFO(node->get_logger(), "Create service named /ack...");
    auto srv = node->create_service<std_srvs::srv::Empty>(
        "ack", [](const std_srvs::srv::Empty::Request::SharedPtr,
                  std_srvs::srv::Empty::Response::SharedPtr) {});
    while (rclcpp::ok() && node->get_service_names_and_types().count("/ack") == 0) {
      RCLCPP_INFO(node->get_logger(), "Waiting until service is listed in the graph...");
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
  }

  RCLCPP_INFO(node->get_logger(), "Waiting until service is destroyed...");
  node->get_node_graph_interface()->notify_graph_change();

  while (rclcpp::ok() && node->get_service_names_and_types().count("/ack") > 0) {
    // executor->spin_some(); // this works!
    RCLCPP_INFO(node->get_logger(), "Checking service does not exist...");
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }

  executor->cancel();
}