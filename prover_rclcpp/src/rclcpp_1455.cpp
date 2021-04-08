#include <chrono>
#include <condition_variable>
#include <memory>
#include <mutex>
#include <string>
#include <thread>

#include "rclcpp/rclcpp.hpp"

std::future<void> spin_executor_async(
  std::shared_ptr<rclcpp::Node> node, rclcpp::Executor &executor)
{
  // We create a timer that will be called as part of the spin
  // this function won't return until the executor is spinning and the timer is called
  std::atomic<bool> timer_called{false};
  std::function<void()> timer_called_lambda =
    [&](){ std::cout << " timer called " << std::endl; timer_called = true; };
  auto timer = node->create_wall_timer(std::chrono::nanoseconds(1), timer_called_lambda);
  executor.add_node(node);
  auto executor_spin_future = std::async(
    std::launch::async, [&executor]() -> void {
      executor.spin();
    });

  while (!timer_called)
  {
    std::this_thread::sleep_for(std::chrono::nanoseconds(1));
  }
  timer.reset();
  executor.remove_node(node);

  return executor_spin_future;
}

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("test");
  auto executor = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
  auto executor_spin_future = spin_executor_async(node, *executor);
  //(void) executor_spin_future.get();
  executor->cancel();

  //rclcpp::shutdown();
  return EXIT_SUCCESS;
}