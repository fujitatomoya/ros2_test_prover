#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

using std::placeholders::_1;

class MinimalSubscriber : public rclcpp::Node
{
public:
  MinimalSubscriber()
    : Node("minimal_subscriber")
    {
      subscription_ = this->create_subscription<std_msgs::msg::String>(
        "topic", 1, std::bind(&MinimalSubscriber::topic_callback, this, _1));
      last_time =  rclcpp::Node::now();
    }

private:
  void topic_callback(const std_msgs::msg::String::SharedPtr msg)
    {
      (void) msg;
      auto time_now = rclcpp::Node::now();
      auto time_passed = time_now - last_time;
      RCLCPP_INFO(this->get_logger(), "%f ms passed since last message",
                  time_passed.seconds() * 1000);
      last_time = time_now;
    }
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
  rclcpp::Time last_time;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MinimalSubscriber>());
  rclcpp::shutdown();
  return 0;
}
