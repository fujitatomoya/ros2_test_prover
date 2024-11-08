#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

using namespace std::chrono_literals;

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("test");
  auto log = node->get_logger();
  rclcpp::executors::SingleThreadedExecutor executor;
  auto spinThread = std::thread([&] {
    executor.add_node(node);
    executor.spin();
    executor.remove_node(node);
  });

  while (!executor.is_spinning()) {
    std::this_thread::sleep_for(1ms);
  }

  {
    RCLCPP_INFO(log, "BEGIN");
    auto cg = node->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    auto timer = node->create_timer(1s, [&log] { RCLCPP_INFO(log, "timer"); }, cg);  // SIGSEGV
    // auto timer = node->create_timer(1s, [&log] { RCLCPP_INFO(log, "timer"); }, std::move(cg)); // OK
    std::this_thread::sleep_for(100ms);  // wait a bit for the executor to catch up
    RCLCPP_INFO(log, "END");             // timer and cg go out of scope here
  }

  executor.cancel();
  if (spinThread.joinable()) {
    spinThread.join();
  }
  rclcpp::shutdown();
}
