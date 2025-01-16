#include <cstdio>

#include "rclcpp/rclcpp.hpp"

static constexpr const uint8_t EXAMPLE_ARR[8]{ 0x00, 0x13, 0xA2, 0x00, 0x41, 0x5C, 0x61, 0x86 };

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  auto node = rclcpp::Node::make_shared("my_node");
  auto param_desc = rcl_interfaces::msg::ParameterDescriptor{};

  param_desc.description = "XBee Due address";
  auto dueAddrParm = node->declare_parameter(
      "byte_arr_param",
      std::vector<uint8_t>(EXAMPLE_ARR,
                           EXAMPLE_ARR + sizeof(EXAMPLE_ARR) / sizeof(EXAMPLE_ARR[0])),
      param_desc);
  auto p = node->get_parameter("byte_arr_param");
  RCLCPP_INFO_STREAM(node->get_logger(), p.get_name() << ":=" << p.value_to_string() << " (initial)");

  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
