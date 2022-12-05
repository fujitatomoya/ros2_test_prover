#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

using namespace std::chrono_literals;

class MinimalPublisher : public rclcpp::Node
{
public:
  MinimalPublisher(rclcpp::NodeOptions node_options)
  : Node("minimal_publisher", node_options), count_(0)
  {
    publisher_ = this->create_publisher<std_msgs::msg::String>("topic", 10);
    auto timer_callback =
      [this]() -> void {
        auto message = std_msgs::msg::String();
        message.data = "Hello, deadlock! " + std::to_string(this->count_++);
        this->publisher_->publish(message);
      };
    timer_ = this->create_wall_timer(1ms, timer_callback);
  }

private:
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
  size_t count_;
};

class MinimalSubscriber : public rclcpp::Node
{
public:
  MinimalSubscriber(int node_number, rclcpp::NodeOptions node_options)
  : Node("node_"+std::to_string(node_number), node_options)
  {
    subscription_ = this->create_subscription<std_msgs::msg::String>(
      "topic", 10, [this](std_msgs::msg::String::UniquePtr msg) { (void)msg; });
  }

  void set_on_new_message_callback()
  {
    auto dummy_cb = [](size_t arg) {(void)arg;};
    subscription_->set_on_new_message_callback(dummy_cb);
  }

private:
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
};

int main(int argc, char * argv[])
{
  for (int i=0; i <100; i++)
  {
    rclcpp::init(argc, argv);

    rclcpp::NodeOptions node_options = rclcpp::NodeOptions();
    node_options.use_intra_process_comms(false);
    node_options.start_parameter_services(false);
    node_options.start_parameter_event_publisher(false);

    auto node_publisher = std::make_shared<MinimalPublisher>(node_options);
    auto pub_executor = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
    pub_executor->add_node(node_publisher);
    std::thread pub_thread([&](){ pub_executor->spin(); });

    for (int i=0; i <10; i++)
    {
      auto node_subscription = std::make_shared<MinimalSubscriber>(i, node_options);
      node_subscription->set_on_new_message_callback();
      std::this_thread::sleep_for(5ms);
    }

    std::this_thread::sleep_for(std::chrono::seconds(1));
    rclcpp::shutdown();
    pub_thread.join();
  }

  return 0;
}
