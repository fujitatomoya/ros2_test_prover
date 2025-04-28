#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <fstream>
#include <iostream>
int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("fuzz_node_chatter");
    try {
        auto pub = node->create_publisher<std_msgs::msg::String>("/chatter", 10);
        std_msgs::msg::String msg;
        std::ifstream fin(argv[1]);
        if (!fin) {
            std::cerr << "Failed to open input file." << std::endl;
            rclcpp::shutdown();
            return 0;
        }
        std::getline(fin, msg.data);
        pub->publish(msg);
    } catch (const rclcpp::exceptions::RCLError &e) {
        std::cerr << "RCLError: " << e.what() << std::endl;
        rclcpp::shutdown();
        return 0;
    } catch (const std::exception &e) {
        std::cerr << "std::exception: " << e.what() << std::endl;
        rclcpp::shutdown();
        return 0;
    }
    rclcpp::shutdown();
    return 0;
}