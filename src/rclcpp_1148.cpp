#include <chrono>
#include <condition_variable>
#include <memory>
#include <mutex>
#include <string>
#include <thread>

#include "rclcpp/rclcpp.hpp"

using namespace std::chrono_literals;

class MultiTimer final : public rclcpp::Node
{
public:
  MultiTimer() : Node("multi_timer")
  {
    timer_ = this->create_wall_timer(
      2s, std::bind(&MultiTimer::timer_callback, this));
  }

private:
  void timer_callback()
  {
    RCLCPP_INFO(this->get_logger(), "Timer executed!");
  }

  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::executors::MultiThreadedExecutor executor;
  auto multi_timer = std::make_shared<MultiTimer>();
  executor.add_node(multi_timer);
  executor.spin();
  rclcpp::shutdown();
  return 0;
}