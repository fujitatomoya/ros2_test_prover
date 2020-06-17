#include <cinttypes>
#include <chrono>
#include <iostream>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "rcutils/cmdline_parser.h"

#include "example_interfaces/srv/add_two_ints.hpp"

using namespace std::chrono_literals;

void print_usage()
{
  printf("Usage for add_two_ints_client app:\n");
  printf("add_two_ints_client [-s service_name] [-h]\n");
  printf("options:\n");
  printf("-h : Print this help function.\n");
  printf("-s service_name : Specify the service name for this client. Defaults to add_two_ints.\n");
}

class ClientNode : public rclcpp::Node
{
public:
  explicit ClientNode(const std::string & service_name)
    : Node("add_two_ints_client")
    {
      client_ = create_client<example_interfaces::srv::AddTwoInts>(service_name);

      // Queue an asynchronous service request that will be sent once `spin` is called on the node.
      queue_async_request();
    }

  void queue_async_request()
    {
      while (!client_->wait_for_service(1s)) {
        if (!rclcpp::ok()) {
          RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for the service. Exiting.");
          return;
        }
        RCLCPP_INFO(this->get_logger(), "service not available, waiting again...");
      }
      auto request = std::make_shared<example_interfaces::srv::AddTwoInts::Request>();
      request->a = 2;
      request->b = 3;

      // We give the async_send_request() method a callback that will get executed once the response
      // is received.
      // This way we can return immediately from this method and allow other work to be done by the
      // executor in `spin` while waiting for the response.
      using ServiceResponseFuture =
        rclcpp::Client<example_interfaces::srv::AddTwoInts>::SharedFutureWithRequest;
        //rclcpp::Client<example_interfaces::srv::AddTwoInts>::SharedFuture;
      auto response_received_callback = [this](ServiceResponseFuture future) {
        auto result = future.get();
        printf("Callback: Getting logger (result = %ld)\n", result.second->sum);
        rclcpp::Logger logger = this->get_logger();
        printf("Callback: Using logger (result = %ld)\n", result.second->sum);
        //RCLCPP_INFO(logger, "Result received! (result = %ld)", result.second->sum);
        printf("Callback: Shutting down\n");
        rclcpp::shutdown();
      };
      auto future_result = client_->async_send_request(request, response_received_callback);
    }

private:
  rclcpp::Client<example_interfaces::srv::AddTwoInts>::SharedPtr client_;
};

int main(int argc, char ** argv)
{
  // Force flush of the stdout buffer.
  // This ensures a correct sync of all prints
  // even when executed simultaneously within the launch file.
  setvbuf(stdout, NULL, _IONBF, BUFSIZ);

  if (rcutils_cli_option_exist(argv, argv + argc, "-h")) {
    print_usage();
    return 0;
  }

  rclcpp::init(argc, argv);

  auto service_name = std::string("add_two_ints");
  char * cli_option = rcutils_cli_get_option(argv, argv + argc, "-s");
  if (nullptr != cli_option) {
    service_name = std::string(cli_option);
  }

  auto node = std::make_shared<ClientNode>(service_name);
  rclcpp::spin(node);

  rclcpp::shutdown();
  return 0;
}
