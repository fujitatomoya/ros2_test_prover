#ifndef MY_STRING_HPP
#define MY_STRING_HPP

#include "rclcpp/rclcpp.hpp"
#include "prover_interfaces/msg/test_msg.hpp"

template <>
struct rclcpp::TypeAdapter<
    std::string,
    prover_interfaces::msg::TestMsg>
{
  using is_specialized = std::true_type;
  using custom_type = std::string;
  using ros_message_type = prover_interfaces::msg::TestMsg;

  static void
  convert_to_ros_message(
      const custom_type &source,
      ros_message_type &destination)
  {
    std::memcpy(&destination.name[0], source.data(), source.size());
    destination.name_size = source.size();
  }

  static void
  convert_to_custom(
      const ros_message_type &source,
      custom_type &destination)
  {
    destination.resize(source.name_size);
    std::memcpy(destination.data(), &source.name[0], source.name_size);
  }
};

RCLCPP_USING_CUSTOM_TYPE_AS_ROS_MESSAGE_TYPE(std::string, prover_interfaces::msg::TestMsg);

#endif // !MY_STRING_HPP