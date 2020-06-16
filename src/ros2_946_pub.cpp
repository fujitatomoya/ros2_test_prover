#include <string>
#include <functional>

#include "std_msgs/msg/string.hpp"
#include "rclcpp/rclcpp.hpp"

using namespace std::chrono_literals;

class MinimalPublisher : public rclcpp::Node
{
public:
  MinimalPublisher()
    : Node("minimal_publisher")
    {
      data=random_string(10000000); // generate a ~10MB string
      publisher_ = this->create_publisher<std_msgs::msg::String>("topic", 1);
      timer_ = this->create_wall_timer(
        100ms, std::bind(&MinimalPublisher::timer_callback, this));
    }

private:
  void timer_callback()
    {
      auto message = std_msgs::msg::String();
      message.data = data;
      publisher_->publish(message);
    }
  std::string random_string( size_t length )
    {
      auto randchar = []() -> char
        {
          const char charset[] =
          "0123456789"
          "ABCDEFGHIJKLMNOPQRSTUVWXYZ"
          "abcdefghijklmnopqrstuvwxyz";
          const size_t max_index = (sizeof(charset) - 1);
          return charset[ rand() % max_index ];
        };
      std::string str(length,0);
      std::generate_n( str.begin(), length, randchar );
      return str;
    }
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
  std::string data;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MinimalPublisher>());
  rclcpp::shutdown();
  return 0;
}
