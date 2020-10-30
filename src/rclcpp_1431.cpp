#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/qos.hpp"
#include "std_msgs/msg/string.hpp"

#define LEN_SET (200 * 1024)

using namespace std::chrono_literals;

/*
 * This example creates a subclass of Node and uses std::bind() to register a
 * member function as a callback from the timer.
 */

class MinimalPublisher : public rclcpp::Node
{
public:
  MinimalPublisher()
  : Node("minimal_publisher"), count_(0)
  {
    publisher_ = this->create_publisher<std_msgs::msg::String>("topic", 10);
    timer_ = this->create_wall_timer(100ms, std::bind(&MinimalPublisher::timer_callback, this));
  }

private:
  void timer_callback()
  {
    auto message = std_msgs::msg::String();
    std::string tmp(LEN_SET, 'a');
    message.data = tmp;
    RCLCPP_INFO(this->get_logger(), "Publishing data...'", message.data.c_str());
    publisher_->publish(message);
    static int64_t old = 0;
    auto tt = std::chrono::high_resolution_clock::now().time_since_epoch().count();
    std::cout << "time in nano seconds " << tt - old << std::endl;
    old = tt;
  }

  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
  size_t count_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MinimalPublisher>());
  rclcpp::shutdown();

  return 0;
}