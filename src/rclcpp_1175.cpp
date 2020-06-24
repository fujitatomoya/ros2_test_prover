#include <cstdlib>
#include <functional>
#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/set_bool.hpp>
#include <std_srvs/srv/trigger.hpp>

using namespace std::placeholders;
using namespace std::chrono_literals;

using namespace std_srvs::srv;

rclcpp::Node::SharedPtr g_node = nullptr;
rclcpp::Service<Trigger>::SharedPtr g_dynsrv = nullptr;

void handle_dynamic_service(
  const std::shared_ptr<rmw_request_id_t> request_header,
  const std::shared_ptr<Trigger::Request> request,
  const std::shared_ptr<Trigger::Response> response)
{
  (void) request_header;
  (void) request;
  RCLCPP_INFO(g_node->get_logger(), "trigger called!");
  response->success = true;
}

void handle_enable_service(
  const std::shared_ptr<rmw_request_id_t> request_header,
  const std::shared_ptr<SetBool::Request> request,
  const std::shared_ptr<SetBool::Response> response)
{
  (void) request_header;
  if(request->data)
  {
    if(g_dynsrv)
    {
      RCLCPP_INFO(g_node->get_logger(), "already enabled");
      response->success = false;
      response->message = "already enabled";
    }
    else
    {
      RCLCPP_INFO(g_node->get_logger(), "enabling dynamic service");
      g_dynsrv = g_node->create_service<Trigger>("dynamic_service", handle_dynamic_service);
      response->success = true;
    }
  }
  else
  {
    if(g_dynsrv)
    {
      RCLCPP_INFO(g_node->get_logger(), "disabling dynamic service");
      g_dynsrv = nullptr;
      response->success = true;
    }
    else
    {
      RCLCPP_INFO(g_node->get_logger(), "already disabled");
      response->success = false;
      response->message = "already disabled";
    }
  }
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  g_node = rclcpp::Node::make_shared("test_dynamic_service");
  auto enable_server = g_node->create_service<SetBool>("enable", handle_enable_service);
  rclcpp::spin(g_node);
  rclcpp::shutdown();
  g_node = nullptr;
  return 0;
}
