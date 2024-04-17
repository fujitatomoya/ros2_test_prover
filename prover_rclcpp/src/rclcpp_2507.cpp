#include "rclcpp/rclcpp.hpp"

int main(void)
{
  RCLCPP_INFO_STREAM(rclcpp::get_logger(""), "Negative rclcpp::Time with int64_t ns");
  const auto negative_ns = rclcpp::Time(-1);
  RCLCPP_INFO_STREAM(rclcpp::get_logger(""), "Negative rclcpp::Time with int32_t s, uint32_t ns");
  const auto negative_s_ns = rclcpp::Time(-1, 0);
  return 0;
}