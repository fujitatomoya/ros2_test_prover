#include "rclcpp/rclcpp.hpp"
#include <string>

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::Node mynode("name");
    mynode.declare_parameter<std::string>("myparam");
    rclcpp::shutdown();
}