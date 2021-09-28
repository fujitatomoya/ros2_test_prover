#include <cinttypes>
#include <memory>

#include "example_interfaces/srv/add_two_ints.hpp"
#include "rclcpp/rclcpp.hpp"

using AddTwoInts = example_interfaces::srv::AddTwoInts;
rclcpp::Node::SharedPtr service_node = nullptr;
rclcpp::Node::SharedPtr client_node = nullptr;

using Executor = rclcpp::executors::SingleThreadedExecutor;

void handle_service(
  const std::shared_ptr<rmw_request_id_t> request_header,
  const std::shared_ptr<AddTwoInts::Request> request,
  const std::shared_ptr<AddTwoInts::Response> response)
{
  (void)request_header;
  response->sum = request->a + request->b;
  RCLCPP_INFO(
    service_node->get_logger(),
    "Handle service. Request: %" PRId64 " + %" PRId64 " = %" PRId64, request->a, request->b, response->sum);
}

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

  // Node options
  rclcpp::NodeOptions node_options = rclcpp::NodeOptions();
  node_options.use_intra_process_comms(false);
  node_options.start_parameter_services(false);
  node_options.start_parameter_event_publisher(false);

  // Create nodes with node options
  service_node = rclcpp::Node::make_shared("service", node_options);
  client_node = rclcpp::Node::make_shared("client", node_options);

  // Client/Service options
  rmw_qos_profile_t qos_profile = rmw_qos_profile_services_default;
  qos_profile.history = RMW_QOS_POLICY_HISTORY_KEEP_LAST;
  std::cout << "Setting OoS depth = 1 for client and service" << std::endl;
  qos_profile.depth = 1;

  // Create client and service with options
  auto client = client_node->create_client<AddTwoInts>("add_two_ints", qos_profile);
  auto server = service_node->create_service<AddTwoInts>("add_two_ints", handle_service, qos_profile);

  // Create separate executors for client and service nodes to better show the issue
  auto service_executor = std::make_shared<Executor>();
  service_executor->add_node(service_node);
  auto client_executor = std::make_shared<Executor>();
  client_executor->add_node(client_node);

  // Send requests (depth is set to 1, so we should only have a single client request)
  for (int i=0; i <= 15; i++) {
    auto request = std::make_shared<AddTwoInts::Request>();
    request->a = i;
    request->b = i;

    std::function<void(
    typename rclcpp::Client<AddTwoInts>::SharedFuture future)> callback_function = std::bind(
        &client_callback,
        request,
        std::placeholders::_1
    );

    std::cout << "Client: async_send_request number: " << i << std::endl;
    client->async_send_request(request, callback_function);
  }

  char a;
  std::cout << "Press key to spin the Server" << std::endl;
  std::cin >> a;

  /******** SERVICE EXECUTOR THREAD *********************************************/
  std::thread service_spin_thread([=](){
      std::cout << "service_executor->spin()" << std::endl;
      service_executor->spin();
  });
  service_spin_thread.detach();
  /********************************************************************************************/

  std::cout << "Press key to spin the Client" << std::endl;
  std::cin >> a;

  /******** CLIENT EXECUTOR THREAD ***********************************************/
  std::thread spin_thread([=](){
      std::cout << "client_executor->spin()" << std::endl;
      client_executor->spin();
  });
  spin_thread.detach();
  /********************************************************************************************/

  std::cin >> a;
  std::cout << "rclcpp::shutdown()" << std::endl;

  rclcpp::shutdown();
  service_node = nullptr;
  client_node = nullptr;
  return 0;
}
