/**
 * Reproducer for https://github.com/ros2/rclcpp/issues/3117
 *
 * ServerGoalHandle::publish_feedback() does not validate goal state.
 * After calling succeed(), the server can still publish feedback without
 * any exception or warning, violating the documented API contract.
 *
 * The documentation says publish_feedback() "throws std::runtime_error
 * if the goal is in any state besides executing", but no validation occurs.
 *
 * Run:
 *   Terminal 1: ros2 run prover_rclcpp rclcpp_3117_action_server
 *   Terminal 2: ros2 run prover_rclcpp rclcpp_3117_action_client
 */

#include <functional>
#include <memory>
#include <thread>

#include "example_interfaces/action/fibonacci.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

class FibonacciActionServer : public rclcpp::Node
{
public:
  using Fibonacci = example_interfaces::action::Fibonacci;
  using GoalHandleFibonacci = rclcpp_action::ServerGoalHandle<Fibonacci>;

  explicit FibonacciActionServer()
  : Node("rclcpp_3117_action_server")
  {
    using namespace std::placeholders;

    action_server_ = rclcpp_action::create_server<Fibonacci>(
      this,
      "fibonacci",
      std::bind(&FibonacciActionServer::handle_goal, this, _1, _2),
      std::bind(&FibonacciActionServer::handle_cancel, this, _1),
      std::bind(&FibonacciActionServer::handle_accepted, this, _1));
  }

private:
  rclcpp_action::Server<Fibonacci>::SharedPtr action_server_;

  rclcpp_action::GoalResponse handle_goal(
    const rclcpp_action::GoalUUID &,
    std::shared_ptr<const Fibonacci::Goal> goal)
  {
    RCLCPP_INFO(this->get_logger(), "Received goal request with order %d", goal->order);
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse handle_cancel(
    const std::shared_ptr<GoalHandleFibonacci>)
  {
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void handle_accepted(const std::shared_ptr<GoalHandleFibonacci> goal_handle)
  {
    std::thread{
      std::bind(&FibonacciActionServer::execute, this, std::placeholders::_1),
      goal_handle
    }.detach();
  }

  void execute(const std::shared_ptr<GoalHandleFibonacci> goal_handle)
  {
    auto result = std::make_shared<Fibonacci::Result>();
    result->sequence = {0, 1, 1, 2, 3};

    // Complete the goal
    goal_handle->succeed(result);
    RCLCPP_INFO(this->get_logger(), "Goal succeeded (terminal state)");

    // BUG: publish_feedback() after succeed() should throw std::runtime_error
    // per documentation, but it silently succeeds.
    auto feedback = std::make_shared<Fibonacci::Feedback>();
    feedback->sequence = {99, 99, 99};
    try {
      goal_handle->publish_feedback(feedback);
      RCLCPP_ERROR(this->get_logger(),
        "BUG: publish_feedback() did NOT throw after succeed()! (rclcpp#3117)");
    } catch (const std::runtime_error & e) {
      RCLCPP_INFO(this->get_logger(),
        "FIXED: publish_feedback() threw: %s", e.what());
    }
  }
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<FibonacciActionServer>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
