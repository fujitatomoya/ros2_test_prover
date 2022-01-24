#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>


class MinimalPublisher: public rclcpp::Node {
public:
  // constructor
  MinimalPublisher()
  : Node("minimal_publisher"){
    publisher_ = this->create_publisher<std_msgs::msg::String>("topic", rclcpp::QoS(1));
    timer_ = this->create_wall_timer(
          std::chrono::seconds(1),
          std::bind(
            &MinimalPublisher::timer_callback,
            this
            )
          );
  }

private:
  // method
  void timer_callback(){
    std::string data;
    for (uint8_t c = 120; c < 130; c++) data.push_back(c);
    auto message = std_msgs::msg::String();
    message.data = std::string("Hello, ") + data + std::string("!");
    publisher_->publish(message);
    RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
    return;
  }

  // attributes
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
};

int main(int argc, char * argv[]){
  rclcpp::init(argc, argv);
  auto pub = std::make_shared<MinimalPublisher>();
  rclcpp::spin(pub);
  rclcpp::shutdown();
  return 0;
}