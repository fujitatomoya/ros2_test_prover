#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

class TestNode : public rclcpp::Node
{
public:
TestNode()
  : Node("test_node")
  {
    this->declare_parameter<bool>("create_sub", false);
    parameter_callback_handle_ = this->add_on_set_parameters_callback(
      std::bind(&TestNode::on_parameter_change, this, std::placeholders::_1));
  }

private:
  rcl_interfaces::msg::SetParametersResult
  on_parameter_change(const std::vector<rclcpp::Parameter> &parameters)
  {
    for (const auto &parameter : parameters) {
      if (parameter.get_name() == "create_sub" &&
        parameter.get_type() == rclcpp::ParameterType::PARAMETER_BOOL)
      {
        if (parameter.as_bool()) {
          if (!subscription_) {
            rclcpp::SubscriptionOptions options;
#if 1 // This disallows us to create the subscription via parameter callback
            options.qos_overriding_options = rclcpp::QosOverridingOptions(
              {
                rclcpp::QosPolicyKind::Depth,
                rclcpp::QosPolicyKind::Durability,
                rclcpp::QosPolicyKind::History,
                rclcpp::QosPolicyKind::Reliability,
              });
#endif
            subscription_ = this->create_subscription<std_msgs::msg::String>(
              "chatter",
              rclcpp::QoS(10),
              std::bind(&TestNode::topic_callback, this, std::placeholders::_1),
              options);
            RCLCPP_INFO(this->get_logger(), "Subscription created.");
          }
        } else {
          subscription_.reset();
          RCLCPP_INFO(this->get_logger(), "Subscription destroyed.");
        }
      }
    }
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;
    return result;
  }

  void topic_callback(const std_msgs::msg::String::SharedPtr msg)
  {
    RCLCPP_INFO(this->get_logger(), "I heard: '%s'", msg->data.c_str());
  }

  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
  OnSetParametersCallbackHandle::SharedPtr parameter_callback_handle_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<TestNode>();

  rclcpp::spin(node);
  rclcpp::shutdown();

  return 0;
}