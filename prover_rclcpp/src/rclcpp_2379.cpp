#include "rclcpp/rclcpp.hpp"
#include "rcl_interfaces/msg/set_parameters_result.hpp"

#include "std_msgs/msg/string.hpp"

class TestParams : public rclcpp::Node
{
public:
  rcl_interfaces::msg::SetParametersResult
  onSetParametersCallback(const std::vector<rclcpp::Parameter> &parameters)
  {
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = false;
    result.reason = "failure";
    for (const auto &parameter : parameters)
    {
      if (parameter.get_name() == "switch" && parameter.get_type() == rclcpp::ParameterType::PARAMETER_BOOL)
      {
        RCLCPP_INFO(this->get_logger(), "Parameter %s changed: %d", parameter.get_name().c_str(), parameter.as_bool());
        result.successful = true;
        result.reason = "success";
      }
    }
    return result;
  }

  void
  postSetParametersCallback(const std::vector<rclcpp::Parameter> &parameters)
  {
    for (const auto & param : parameters) {
      if (param.get_name() == "switch") {
        if (param.as_bool()) {
          RCLCPP_INFO(this->get_logger(), "Creating Subscription");
          try {
            this->create_sub();
          } catch (const std::runtime_error & err) {
            RCLCPP_ERROR(this->get_logger(), "Caught Exception with %s", err.what());
          }
        } else {
          RCLCPP_INFO(this->get_logger(), "Destroying Subscription");
          this->destroy_sub();
        }
      }
    }
  }

  void
  create_sub()
  {
    const rclcpp::QoS qos(1);
    auto options = rclcpp::SubscriptionOptions();
    options.qos_overriding_options = rclcpp::QosOverridingOptions::with_default_policies();
    auto callback = [](std_msgs::msg::String::ConstSharedPtr) {};
    auto subscription_ = this->create_subscription<std_msgs::msg::String>("topic", qos, callback, options);
  }

  void
  destroy_sub()
  {
    subscription_.reset();
  }

  TestParams() : Node("test_params")
  {
    // declare parameter 1st, so it will not be undeclared parameter
    this->declare_parameter<bool>("switch", false);

    // register callbacks
    on_set_param_callback_handle_ = this->add_on_set_parameters_callback(
        std::bind(&TestParams::onSetParametersCallback, this, std::placeholders::_1));
    post_set_param_callback_handle_ = this->add_post_set_parameters_callback(
        std::bind(&TestParams::postSetParametersCallback, this, std::placeholders::_1));
  }

private:
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr on_set_param_callback_handle_;
  rclcpp::node_interfaces::PostSetParametersCallbackHandle::SharedPtr post_set_param_callback_handle_;

  std::shared_ptr<std_msgs::msg::String> subscription_{nullptr};
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<TestParams>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}