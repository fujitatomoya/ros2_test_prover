#include <chrono>
#include <thread>

#include <rclcpp/rclcpp.hpp>

class TestNode : public rclcpp::Node {
public:
  TestNode()
      : Node("test") {}

  void subscribe(const std::string& topic, const std::string& dataType) {
    RCLCPP_INFO(get_logger(), "Subscribing to %s (%s)", topic.c_str(), dataType.c_str());

    // Create a temporary callback group. Will be destroyed when the variable gets out of scope.
    // Note that this callback group is unused.
    // When the following line is commented, everything works as expected.
    auto cbg = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

    _subscription = this->create_generic_subscription(
      topic, dataType, 10, [this](std::shared_ptr<rclcpp::SerializedMessage> msg) {
        RCLCPP_INFO(get_logger(), "Message with %lu bytes received", msg->size());
      });
  }

  void unsubscribe() {
    RCLCPP_INFO(get_logger(), "Unsubscribing...");
    _subscription.reset();
  }

  bool isSubscribed() const {
    return static_cast<bool>(_subscription);
  }

private:
  rclcpp::GenericSubscription::SharedPtr _subscription;
};

int main(int argc, char** argv) {
  if (argc < 3) return -1;

  const std::string topic = argv[1];
  const std::string dataType = argv[2];

  rclcpp::init(argc, argv);
  rclcpp::executors::MultiThreadedExecutor executor;
  auto node = std::make_shared<TestNode>();
  executor.add_node(node);

  std::thread toggleSubscriptionThread([&node, &topic, &dataType]() {
    while (rclcpp::ok()) {
      if (node->isSubscribed()) {
        node->unsubscribe();
      } else {
        node->subscribe(topic, dataType);
      }
      std::this_thread::sleep_for(std::chrono::seconds(3));
    }
  });

  executor.spin();
  toggleSubscriptionThread.join();
  rclcpp::shutdown();
}