#include <cinttypes>
#include <memory>

#include "example_interfaces/srv/add_two_ints.hpp"
#include "rclcpp/rclcpp.hpp"

using AddTwoInts = example_interfaces::srv::AddTwoInts;
rclcpp::Node::SharedPtr client_node = nullptr;

void client_callback(
  std::shared_ptr<typename AddTwoInts::Request> request,
  typename rclcpp::Client<AddTwoInts>::SharedFuture result_future)
{
    auto result = result_future.get();
    RCLCPP_INFO(
      client_node->get_logger(),
      "Client callback. Result of %" PRId64 " + %" PRId64 " = %" PRId64, request->a, request->b, result->sum);
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  rclcpp::NodeOptions node_options = rclcpp::NodeOptions();
  client_node = rclcpp::Node::make_shared("client", node_options);

  auto client = client_node->create_client<AddTwoInts>("add_two_ints", rclcpp::ServicesQoS());

  auto request = std::make_shared<AddTwoInts::Request>();
  request->a = 1;
  request->b = 2;

  std::cout << "Client: async_send_request call !!!" << std::endl;
  auto future_result = client->async_send_request(request);

  if (rclcpp::spin_until_future_complete(client_node, future_result, std::chrono::seconds(3)) !=
    rclcpp::FutureReturnCode::SUCCESS)
  {
    return EXIT_FAILURE;
  }
  RCLCPP_INFO(
    client_node->get_logger(),
    "Result(1st) = %" PRId64, future_result.get()->sum);
  RCLCPP_INFO(
    client_node->get_logger(),
    "Result(2nd) = %" PRId64, future_result.get()->sum);

  rclcpp::shutdown();
  client_node = nullptr;
  return EXIT_SUCCESS;
}
