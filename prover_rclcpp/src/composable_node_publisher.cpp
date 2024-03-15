#include <chrono>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_components/register_node_macro.hpp"
#include "std_msgs/msg/string.hpp"

namespace prover_rclcpp
{

class ComposableNodePublisher : public rclcpp::Node
{
public:
  explicit ComposableNodePublisher(const rclcpp::NodeOptions& options)
    : rclcpp::Node("publisher", options)
  {
    double pub_rate = this->declare_parameter<double>("publish_rate", 1.0);
    pub_ = this->create_publisher<std_msgs::msg::String>("/chatter", rclcpp::QoS(10));

    auto cb = [this] () {
      static int count = 0;
      RCLCPP_DEBUG(this->get_logger(), "%d", count);
      auto msg = std::make_unique<std_msgs::msg::String>();
      msg->data = std::to_string(count);
      pub_->publish(std::move(msg));
      ++count;
    };
    timer_ = rclcpp::create_timer(
        this, this->get_clock(),
        rclcpp::Duration(std::chrono::duration<double>(1.0 / pub_rate)),
        std::move(cb));
    RCLCPP_INFO_STREAM(this->get_logger(), "End of constructor.");
  }

  ~ComposableNodePublisher() = default;

private:
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_;
};

}  // namespace prover_rclcpp

RCLCPP_COMPONENTS_REGISTER_NODE(prover_rclcpp::ComposableNodePublisher);
