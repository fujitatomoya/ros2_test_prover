#include <chrono>
#include <condition_variable>
#include <memory>
#include <mutex>
#include <string>
#include <thread>

#include "rclcpp/rclcpp.hpp"

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("test");
  auto executor = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
  executor->add_node(node);
  auto executor_spin_future = std::async(
      std::launch::async, [&executor]() -> void {
      std::this_thread::sleep_for(std::chrono::microseconds(100));
      executor->spin();
    });
  executor->cancel();
  (void) executor_spin_future.get();

  rclcpp::shutdown();
  return EXIT_SUCCESS;
}