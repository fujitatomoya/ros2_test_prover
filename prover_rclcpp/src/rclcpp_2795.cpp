#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

using namespace std::chrono_literals;

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("parent");
  auto subnode = node->create_sub_node("subnode");

  // This generates what():  parameter 'param_name' has already been declared
  //node->declare_parameter("param_name", 0);
  //std::cout << node->get_parameter("param_name").get_value<int>() << std::endl;
  subnode->declare_parameter("param_name", 5);
  std::cout << subnode->get_parameter("param_name").get_value<int>() << std::endl;  // 5

  rclcpp::Parameter param;
  node->get_parameter("param_name", param);
  std::cout << param.get_value<int>() << std::endl;  // 5
  subnode->get_parameter("param_name", param);
  std::cout << param.get_value<int>() << std::endl;  // 5

  int param_int;
  node->get_parameter("param_name", param_int);
  std::cout << param_int << std::endl;  // 5
  subnode->get_parameter("param_name", param_int);
  std::cout << param_int << std::endl;  // 5

  std::cout << node->get_parameter_or("param_name", 666) << std::endl;  // 5
  std::cout << subnode->get_parameter_or("param_name", 666) << std::endl;  // 666

  node->get_parameter_or("param_name", param_int, 333);  // 5
  std::cout << param_int << std::endl;
  subnode->get_parameter_or("param_name", param_int, 666);  // 666
  std::cout << param_int << std::endl;

  rclcpp::shutdown();
}
