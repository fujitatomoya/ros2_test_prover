#include <time.h>
#include <chrono>  
#include <iostream> 
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("test");
    rclcpp::Rate loop_rate(20);

    auto subscriber = node->create_subscription<std_msgs::msg::String>(
      "/test/string", 100,
      [](const std::shared_ptr<std_msgs::msg::String> msg){
        (void) msg;
        std::chrono::time_point<std::chrono::system_clock,std::chrono::milliseconds> tp =
          std::chrono::time_point_cast<std::chrono::milliseconds>(std::chrono::system_clock::now());
        std::time_t timestamp =  tp.time_since_epoch().count(); 
        std::cout << "timestamp: " << timestamp << std::endl;
      }
    );
    while (rclcpp::ok())
    {
        rclcpp::spin_some(node);
        RCLCPP_INFO(node->get_logger(), "spin_some completed, next loop...");
        loop_rate.sleep();
    }
    return 0;
}