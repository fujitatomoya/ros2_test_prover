#include <functional>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/empty.hpp"

// terminal-1) $ ros2 run intraprocess_pubsub intraprocess_pub_sub
// terminal-2) $ ros2 topic pub /goal std_msgs/msg/Empty {} -1

using std::placeholders::_1;

class IntraprocessPubSub : public rclcpp::Node
{
public:
  IntraprocessPubSub() : Node("intraprocess_pub_sub", rclcpp::NodeOptions().use_intra_process_comms(true))
  {
    feedback_publisher_ = this->create_publisher<std_msgs::msg::Empty>("feedback", 1);
    goal_subscription_ = this->create_subscription<std_msgs::msg::Empty>(
        "goal", 10, std::bind(&IntraprocessPubSub::goal_callback, this, _1));
    feedback_subscription_ = this->create_subscription<std_msgs::msg::Empty>(
        "feedback", 10, std::bind(&IntraprocessPubSub::feedback_callback, this, _1));
    local_publisher_ = this->create_publisher<std_msgs::msg::Empty>("local_topic", 1);
    local_subscription_ = this->create_subscription<std_msgs::msg::Empty>(
        "local_topic", 1, std::bind(&IntraprocessPubSub::local_callback, this, _1));
  }

private:
  void goal_callback(const std_msgs::msg::Empty::ConstSharedPtr msg) const
  {
    (void) msg;
    RCLCPP_INFO(rclcpp::get_logger("intraprocess_pub_sub"), "goal_callback()");

    std_msgs::msg::Empty message;
#if 0
    auto message_ptr = std::make_unique<std_msgs::msg::Empty>();
    local_publisher_->publish(std::move(message_ptr));
#else
    local_publisher_->publish(message);
#endif
    feedback_publisher_->publish(message);
  }

  void feedback_callback(const std_msgs::msg::Empty::ConstSharedPtr msg) const
  {
    (void) msg;
    RCLCPP_INFO(rclcpp::get_logger("intraprocess_pub_sub"), "feedback_callback()");
  }

  void local_callback(const std_msgs::msg::Empty::ConstSharedPtr msg) const
  {
    (void) msg;
    RCLCPP_INFO(rclcpp::get_logger("intraprocess_pub_sub"), "local_callback() sometimes this is not called, why?");
  }

  rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr feedback_publisher_;
  rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr goal_subscription_;
  rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr feedback_subscription_;
  rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr local_publisher_;
  rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr local_subscription_;
};

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<IntraprocessPubSub>();

  rclcpp::Executor::UniquePtr executor =
      std::make_unique<rclcpp::executors::MultiThreadedExecutor>(rclcpp::ExecutorOptions(), 0);
  executor->add_node(node->get_node_base_interface());

  executor->spin();

  rclcpp::shutdown();
  return 0;
}
