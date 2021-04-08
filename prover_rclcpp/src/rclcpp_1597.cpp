#include "rclcpp/rclcpp.hpp"
#include "rcl_interfaces/msg/set_parameters_result.hpp"

class TestParams : public rclcpp::Node
{
public:
  rcl_interfaces::msg::SetParametersResult parametersCallback(const std::vector<rclcpp::Parameter> &parameters)
  {
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;
    result.reason = "success";
    for (const auto &parameter : parameters)
    {
      std::cerr << "Parameter callback" << std::endl;
      if (parameter.get_name() == "my_str" && parameter.get_type() == rclcpp::ParameterType::PARAMETER_STRING)
      {
        my_str_ = parameter.as_string();
        RCLCPP_INFO(this->get_logger(), "Parameter 'my_str' changed: %s", my_str_.c_str());
      }
    }
    return result;
  }

  TestParams() : Node("test_params_rclcpp")
  {
    this->declare_parameter("my_str", "default value");
    param_callback_handle_ = this->add_on_set_parameters_callback(std::bind(&TestParams::parametersCallback, this, std::placeholders::_1));
  }

private:
  std::string my_str_;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_callback_handle_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<TestParams>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}