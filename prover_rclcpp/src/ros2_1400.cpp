#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_srvs/srv/empty.hpp"

using namespace std::placeholders;

class MultiThreadedNode : public rclcpp::Node
{
public:
    MultiThreadedNode()
    : Node("multi_threaded_node")
    {
        auto default_qos = rclcpp::QoS(10);
        cb_group_1_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
        cb_group_2_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
        cb_group_3_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

        rclcpp::SubscriptionOptions sub_options;
        sub_options.callback_group = cb_group_1_;

        subscriber_ = create_subscription<std_msgs::msg::String>(
            "chatter",
            default_qos, 
            std::bind(&MultiThreadedNode::subscriber_callback, this, _1),
            sub_options);

        service_ = create_service<std_srvs::srv::Empty>(
            "service",
            std::bind(&MultiThreadedNode::service_callback, this, _1, _2),
            default_qos,
            cb_group_2_);

        timer_ = create_wall_timer(
            std::chrono::seconds(10),
            std::bind(&MultiThreadedNode::timer_callback, this),
            cb_group_3_);
    }
private:
    void subscriber_callback(const std_msgs::msg::String::SharedPtr msg)
    {
        (void)msg;
        RCLCPP_INFO(get_logger(), ".");
    }

    void service_callback(
        const std_srvs::srv::Empty::Request::SharedPtr req,
        const std_srvs::srv::Empty::Response::SharedPtr res)
    {
        (void)req;
        (void)res;
        RCLCPP_INFO(this->get_logger(), "enter service callback");
        for (size_t i = 0; i < 10; i++)
        {
            RCLCPP_INFO(this->get_logger(), "#");
            std::this_thread::sleep_for(std::chrono::seconds(1));
        }
        RCLCPP_INFO(this->get_logger(), "exit service callback");
    }

    void timer_callback()
    {
        RCLCPP_INFO(this->get_logger(), "enter timer callback");
        for (int i = 1; i < 10; ++i) {
            RCLCPP_INFO(this->get_logger(), "+");
            std::this_thread::sleep_for(std::chrono::seconds(1));
        }
        RCLCPP_INFO(this->get_logger(), "exit timer callback");
    }

    rclcpp::CallbackGroup::SharedPtr cb_group_1_;
    rclcpp::CallbackGroup::SharedPtr cb_group_2_;
    rclcpp::CallbackGroup::SharedPtr cb_group_3_;

    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscriber_;
    rclcpp::Service<std_srvs::srv::Empty>::SharedPtr service_;
    rclcpp::TimerBase::SharedPtr timer_;

};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<MultiThreadedNode>();
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);
    executor.spin();

    rclcpp::shutdown();
    return 0;
}