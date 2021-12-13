#include <chrono>
#include <memory>

using namespace std::chrono_literals;

#include "rclcpp/rclcpp.hpp"
#include "prover_interfaces/msg/property.hpp"

class TestPublisher : public rclcpp::Node
{
  public:
    TestPublisher()
    : Node("property_publisher")
    {
      publisher_ = this->create_publisher<prover_interfaces::msg::Property>("property", 10);
      timer_ = this->create_wall_timer(
        1s, std::bind(&TestPublisher::timer_callback, this)); // 1kHz
    }

  private:
    void timer_callback()
    {
      auto message = prover_interfaces::msg::Property();
      message.property = "This is property";
      message.anything = "This is anything";
      publisher_->publish(message);
    }
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<prover_interfaces::msg::Property>::SharedPtr publisher_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    std::shared_ptr<TestPublisher> pub = std::make_shared<TestPublisher>();
    rclcpp::spin(pub);
    rclcpp::shutdown();
    return 0;
}
