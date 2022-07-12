#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_srvs/srv/trigger.hpp>

class CallbackGroupSrvClient: public rclcpp::Node {
public:
  // Constructor
  CallbackGroupSrvClient()
  : Node("callback_group_srv_client"){

    // Init callback group
    callback_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive, false);
    callback_group_executor_thread_ = std::thread([this]() {
      callback_group_executor_.add_callback_group(callback_group_, this->get_node_base_interface());
      callback_group_executor_.spin();
    });

    // Init subscription
    subscription_ = this->create_subscription<std_msgs::msg::Bool>(
          "topic_bool",
          rclcpp::QoS(1),
          std::bind(
            &CallbackGroupSrvClient::sub_callback,
            this,
            std::placeholders::_1
            )
          );

    // Create service client and link it to callback group
    srv_client_ = this->create_client<std_srvs::srv::Trigger>(
          "trigger_srv",
          rclcpp::ServicesQoS(),
          callback_group_
          );
  }

  // Destructor
  ~CallbackGroupSrvClient(){
    callback_group_executor_.cancel();
    callback_group_executor_thread_.join();
  }

private:
  // Methods
  void sub_callback(const std_msgs::msg::Bool::SharedPtr ){
    RCLCPP_INFO_STREAM(this->get_logger(), "Received new msg! ");

    // Send request and wait for future
    auto req = std::make_shared<std_srvs::srv::Trigger::Request>();
    auto future = srv_client_->async_send_request(req);
    RCLCPP_INFO_STREAM(this->get_logger(), "Sending request");
    auto future_status = future.wait_for(std::chrono::seconds(10));

    // Process result
    if (future_status == std::future_status::ready) {
      if (future.get()->success) {
        RCLCPP_INFO_STREAM(this->get_logger(), "Srv suceeded");
      }
      else {
        RCLCPP_INFO_STREAM(this->get_logger(), "Srv failed");
      }
    }
    else if (future_status == std::future_status::timeout) {
      RCLCPP_INFO_STREAM(this->get_logger(), "Response not received within timeout");
    }
    else if (future_status == std::future_status::deferred) {
      RCLCPP_INFO_STREAM(this->get_logger(), "Srv deferred");
    }

    return;
  }

  // Attributes
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr subscription_;
  rclcpp::CallbackGroup::SharedPtr callback_group_;
  rclcpp::executors::SingleThreadedExecutor callback_group_executor_;
  std::thread callback_group_executor_thread_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr srv_client_;
};



int main(int argc, char * argv[]){
  rclcpp::init(argc, argv);
  auto client = std::make_shared<CallbackGroupSrvClient>();
  rclcpp::spin(client);
  rclcpp::shutdown();
  return 0;
}