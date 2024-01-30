#include <memory>
#include "rclcpp/rclcpp.hpp"
#include <chrono>
#include "rclcpp_2417_myString.hpp"

using TopicSub = std::string;

class MinimalSubscriber : public rclcpp::Node
{
public:
  MinimalSubscriber()
      : Node("minimal_subscriber")
  {
    auto callback = [this](const TopicSub &msg)
    {
      std::cout << "msg = " << msg << std::endl;
    };

    subscription_ = this->create_subscription<TopicSub>("topic", 10, callback);
  }

private:
  rclcpp::Subscription<TopicSub>::SharedPtr subscription_;
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MinimalSubscriber>());
  rclcpp::shutdown();
  return 0;
}