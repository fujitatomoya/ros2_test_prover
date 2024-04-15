#include <memory>

#include <rclcpp/context.hpp>
#include <rclcpp/guard_condition.hpp>

int main(int argc, char** argv) {
  auto context = std::make_shared<rclcpp::Context>();
  context->init(argc, argv);
  context->get_sub_context<rclcpp::GuardCondition>(context);
  context->shutdown("shutdown");
  return 0;
}
