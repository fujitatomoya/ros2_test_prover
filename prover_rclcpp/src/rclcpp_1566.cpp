#include <iostream>
#include <memory>

#include "rclcpp/rclcpp.hpp"

static int count = 0;

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("foo_node");

  auto param_change_callback =
    [](const std::vector<rclcpp::Parameter> & parameters)
    {
      for (const auto & parameter : parameters) {
        std::cerr << "Parameter callback: " << count << " " << parameter.get_name() << std::endl;
      }
      count++;
      auto result = rcl_interfaces::msg::SetParametersResult();
      result.successful = true;
      return result;
    };
  auto param_callback_handle = node->add_on_set_parameters_callback(param_change_callback);

  rclcpp::spin(node);

  rclcpp::shutdown();
  return 0;
}
