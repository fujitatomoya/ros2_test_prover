#include <string>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/qos.hpp"
#include "std_msgs/msg/string.hpp"

class MyNode : public rclcpp::Node
{
public:
    MyNode() : rclcpp::Node("my_node")
    {
#if 0 // success, store lvalue to lvalue
       auto node_topics_interface = this->get_node_topics_interface();
       auto pub = rclcpp::create_publisher<std_msgs::msg::String>(
            node_topics_interface,
            "my_topic",
             rclcpp::SensorDataQoS());
#else // failure, rvalue to lvalue reference 
        auto pub = rclcpp::create_publisher<std_msgs::msg::String>(
            this->get_node_topics_interface(),
            "my_topic",
             rclcpp::SensorDataQoS());
#endif
    }
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    std::shared_ptr<MyNode> mynode = std::make_shared<MyNode>();
    rclcpp::shutdown();
}