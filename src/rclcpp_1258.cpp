#include <stdlib.h>
#include <iostream>

#include <rclcpp/rclcpp.hpp>

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto options = rclcpp::InitOptions().auto_initialize_logging(false);
  if (options.auto_initialize_logging()) {
    std::cout << "options.auto_initialize_logging() is true\n";
    return EXIT_FAILURE;
  } else {
    std::cout << "options.auto_initialize_logging() is false\n";
    return EXIT_SUCCESS;
  }
}
