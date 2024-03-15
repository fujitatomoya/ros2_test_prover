#include <algorithm>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_components/register_node_macro.hpp"
#include "std_msgs/msg/string.hpp"

namespace prover_rclcpp
{

using std::placeholders::_1;

class ComposableNodeSubscription : public rclcpp::Node
{
public:
  explicit ComposableNodeSubscription(const rclcpp::NodeOptions& options)
    : rclcpp::Node("subscription", options)
  {
    sub_ = this->create_subscription<std_msgs::msg::String>(
        "/chatter", rclcpp::QoS(10), std::bind(&ComposableNodeSubscription::topicHandler, this, _1));
    RCLCPP_INFO_STREAM(this->get_logger(), "End of constructor.");
  }

  ~ComposableNodeSubscription() = default;

private:
  void topicHandler(const std_msgs::msg::String::ConstSharedPtr msg)
  {
    RCLCPP_DEBUG_STREAM(this->get_logger(), msg->data);
  }

  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_;
};

}  // namespace prover_rclcpp

RCLCPP_COMPONENTS_REGISTER_NODE(prover_rclcpp::ComposableNodeSubscription);
