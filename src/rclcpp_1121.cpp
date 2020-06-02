#include <atomic>
#include <chrono>
#include <iostream>
#include <future>

#include "rclcpp/rclcpp.hpp"
#include "std_srvs/srv/set_bool.hpp"

void runService(const std::string& service_name, std::atomic<bool>& stop)
{
    // Node
    auto rclcpp_node = rclcpp::Node::make_shared("rclcpp_1121_repro_service_node");
    
    // Executor
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(rclcpp_node);
    const auto executor_future = std::async([&executor] {
        executor.spin();
    });

    // Service
    auto rclcpp_service = rclcpp_node->create_service<std_srvs::srv::SetBool>(
        service_name,
        [](const std_srvs::srv::SetBool_Request::SharedPtr request,
            std_srvs::srv::SetBool_Response::SharedPtr response) {
            // Simulating response taking some time
            std::this_thread::sleep_for(std::chrono::milliseconds{500});

            response->success = request->data;
            response->message = "Hi from the service";
        });

    while (rclcpp::ok() && !stop)
    {
        std::this_thread::sleep_for(std::chrono::milliseconds{100});
    }

    rclcpp_service.reset();
    
    executor.cancel();
}

void request(const std::string& service_name, std::atomic<bool>& stop)
{
    // Node
    auto rclcpp_node = rclcpp::Node::make_shared("rclcpp_1121_repro_client_node");
    
    // Executor
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(rclcpp_node);
    const auto executor_future = std::async([&executor] {
        executor.spin();
    });

    // Client
    auto rclcpp_client = rclcpp_node->create_client<std_srvs::srv::SetBool>(service_name);
    auto future_response = std::async([&rclcpp_client, &stop]() -> std::string {
        constexpr std::chrono::milliseconds single_spin_limit{100};

        while (rclcpp::ok() && !stop && !rclcpp_client->wait_for_service(single_spin_limit))
        {
        }

        if (!rclcpp::ok() || stop)
        {
            throw std::runtime_error{"Interrupted"};
        }

        auto request = std::make_shared<std_srvs::srv::SetBool_Request>();
        request->data = true;

        auto rclcpp_request_future = rclcpp_client->async_send_request(request);

        std::future_status future_status{std::future_status::timeout};
        while (!stop && rclcpp::ok() &&
                (future_status = rclcpp_request_future.wait_for(single_spin_limit)) == std::future_status::timeout)
        {
        }

        if (future_status == std::future_status::ready)
        {
            return rclcpp_request_future.get()->message;
        }

        throw std::runtime_error{"Request terminated before receiving the response"};
    });

    const auto response_status = future_response.wait_for(std::chrono::seconds{5});
    
    if (response_status == std::future_status::ready)
    {
        std::cout << "Response received timely: " << future_response.get() << std::endl;
    }
    else
    {
        std::cout << "Response timed out!" << std::endl;
        stop = true;
        future_response.wait();
    }
    
    rclcpp_client.reset();

    executor.cancel();
}

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);

    const std::string service_name = "rclcpp_1121_repro_service";
    std::atomic<bool> stop{false};

    auto service_future = std::async([&]() { runService(service_name, stop); });
    auto client_future = std::async([&]() { request(service_name, stop); });

    client_future.wait();

    stop = true;
    service_future.wait();

    rclcpp::shutdown();

    return 0;
}
