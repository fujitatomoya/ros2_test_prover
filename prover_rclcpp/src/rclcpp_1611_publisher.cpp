#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>


class MinimalPublisher: public rclcpp::Node {
public:
  // constructor
  MinimalPublisher()
  : Node("minimal_publisher"){
    publisher_ = this->create_publisher<std_msgs::msg::Bool>("topic_bool", rclcpp::QoS(1));
    timer_ = this->create_wall_timer(
          std::chrono::seconds(5),
          std::bind(
            &MinimalPublisher::timer_callback,
            this
            )
          );
  }

private:
  // method
  void timer_callback(){
    std_msgs::msg::Bool msg;
    msg.data = false;
    publisher_->publish(msg);
    RCLCPP_INFO_STREAM(this->get_logger(), "Publishing data! ");
    return;
  }

  // attributes
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr publisher_;
};

int main(int argc, char * argv[]){
  rclcpp::init(argc, argv);
  auto pub = std::make_shared<MinimalPublisher>();
  rclcpp::spin(pub);
  rclcpp::shutdown();
  return 0;
}