#include <chrono>
#include <condition_variable>
#include <memory>
#include <mutex>
#include <string>
#include <thread>

#include "rclcpp/rclcpp.hpp"

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  std::cout << "Doing something terrible!\n";
  int *p = 0;
  *p = 0;
  std::cout << "Got away with it\n";
  rclcpp::shutdown();
  return EXIT_SUCCESS;
}