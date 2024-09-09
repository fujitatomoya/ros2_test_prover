#include <atomic>
#include <memory>
#include <thread>

#include <rclcpp/executors/multi_threaded_executor.hpp>
#include <rclcpp/node.hpp>

auto function() -> void
{
    rclcpp::init(0, nullptr);

    rclcpp::executors::MultiThreadedExecutor executor;

    std::atomic_bool stop_spinning{false};
    auto spinner_thread{std::thread([&] {
        while (rclcpp::ok() && !stop_spinning.load()) {
            executor.spin_some();
        }
    })};

    {
        auto node{std::make_shared<rclcpp::Node>("node")};
        executor.add_node(node);
    }

    stop_spinning.store(true);
    spinner_thread.join();

    rclcpp::shutdown();
}

auto main() -> int
{
    while (true) {
        function();
    }

    return 0;
}
