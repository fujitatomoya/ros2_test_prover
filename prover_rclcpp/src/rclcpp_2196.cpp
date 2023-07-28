#include <chrono>
#include <condition_variable>
#include <memory>
#include <mutex>
#include <string>
#include <stdexcept>
#include <thread>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

using namespace std::chrono_literals;

class MinimalPublisher : public rclcpp::Node
{
  public:
    MinimalPublisher()
    : Node("minimal_publisher"), count_(0)
    {
      publisher_ = this->create_publisher<std_msgs::msg::String>("topic", 10);
      timer_ = this->create_wall_timer(
        1s, std::bind(&MinimalPublisher::timer_callback, this));
    }

  private:
    void timer_callback()
    {
      auto message = std_msgs::msg::String();
      message.data = "Hello, world! " + std::to_string(count_++);
      RCLCPP_INFO(this->get_logger(), "Generating runtime error exception");
      throw std::runtime_error("timer callback error!\n");
      RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
      publisher_->publish(message);
    }
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
    size_t count_;
  };

  int main(int argc, char * argv[])
  {
    rclcpp::init(argc, argv);

    //rclcpp::executors::SingleThreadedExecutor exe; // This works
    rclcpp::executors::MultiThreadedExecutor exe; // This does not

    std::shared_ptr<MinimalPublisher> pub = std::make_shared<MinimalPublisher>();
    exe.add_node(pub->get_node_base_interface());

    try {
      exe.spin();
    } catch (const std::runtime_error & err) {
      RCLCPP_INFO(pub->get_logger(), "Caught runtime error exception");
    }
    rclcpp::shutdown();
    return 0;
  }

