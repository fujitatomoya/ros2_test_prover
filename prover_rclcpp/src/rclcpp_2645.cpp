#include <rclcpp/rclcpp.hpp>

using namespace std::chrono_literals;

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("test_timer_starvation");
  auto log = node->get_logger();
  rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), 2u);

  auto cb = node->create_callback_group(rclcpp::CallbackGroupType::Reentrant);

  std::atomic_int timer_one_count{0};
  std::atomic_int timer_two_count{0};

  rclcpp::TimerBase::SharedPtr timer_one;
  rclcpp::TimerBase::SharedPtr timer_two;

  auto timer_one_callback = [&timer_one_count, &timer_two_count]() {
    std::cout << "Timer one callback executed. Count: "
              << timer_one_count.load() << std::endl;

    auto start_time = std::chrono::steady_clock::now();
    while (std::chrono::steady_clock::now() - start_time < 100ms) {
    }

    timer_one_count++;

    auto diff = std::abs(timer_one_count - timer_two_count);

    std::cout << "Difference in counts: " << diff << std::endl;
  };

  auto timer_two_callback = [&timer_one_count, &timer_two_count]() {
    std::cout << "Timer two callback executed. Count: "
              << timer_two_count.load() << std::endl;

    auto start_time = std::chrono::steady_clock::now();
    while (std::chrono::steady_clock::now() - start_time < 100ms) {
    }

    timer_two_count++;

    auto diff = std::abs(timer_one_count - timer_two_count);

    std::cout << "Difference in counts: " << diff << std::endl;
  };

  timer_one = node->create_wall_timer(50ms, timer_one_callback, cb);
  timer_two = node->create_wall_timer(50ms, timer_two_callback, cb);

  executor.add_node(node);
  executor.spin();

  rclcpp::shutdown();
}
