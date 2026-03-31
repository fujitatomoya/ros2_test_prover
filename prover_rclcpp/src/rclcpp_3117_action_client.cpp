/**
 * Reproducer for https://github.com/ros2/rclcpp/issues/3117
 *
 * Simple action client to trigger the server-side bug.
 *
 * Run:
 *   Terminal 1: ros2 run prover_rclcpp rclcpp_3117_action_server
 *   Terminal 2: ros2 run prover_rclcpp rclcpp_3117_action_client
 */

#include <memory>

#include "example_interfaces/action/fibonacci.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

int main(int argc, char * argv[])
{
  using Fibonacci = example_interfaces::action::Fibonacci;

  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("rclcpp_3117_action_client");
  auto client = rclcpp_action::create_client<Fibonacci>(node, "fibonacci");

  if (!client->wait_for_action_server(std::chrono::seconds(10))) {
    RCLCPP_ERROR(node->get_logger(), "Action server not available");
    rclcpp::shutdown();
    return 1;
  }

  auto goal_msg = Fibonacci::Goal();
  goal_msg.order = 5;

  RCLCPP_INFO(node->get_logger(), "Sending goal...");
  auto future = client->async_send_goal(goal_msg);

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  executor.spin_until_future_complete(future);

  RCLCPP_INFO(node->get_logger(), "Done. Check the server output for BUG messages.");
  rclcpp::shutdown();
  return 0;
}
