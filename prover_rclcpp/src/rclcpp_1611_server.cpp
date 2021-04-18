#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_srvs/srv/trigger.hpp>


class CallbackGroupSrvServer: public rclcpp::Node {
public:
  // Constructor
  CallbackGroupSrvServer()
  : Node("callback_group_srv_server"){
    // Init callback group and spin it
    callback_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive, false);
    //callback_group_ = this->create_callback_group(rclcpp::CallbackGroupType::Reentrant, false);
    callback_group_executor_thread_ = std::thread([this]() {
      callback_group_executor_.add_callback_group(callback_group_, this->get_node_base_interface());
      callback_group_executor_.spin();
    });

    // Init service server and link it to callback group
    service_ = this->create_service<std_srvs::srv::Trigger>(
          "trigger_srv",
          std::bind(
            &CallbackGroupSrvServer::srv_callback,
            this,
            std::placeholders::_1,
            std::placeholders::_2)
          );
  }

  // Destructor
  ~CallbackGroupSrvServer(){
    callback_group_executor_.cancel();
    callback_group_executor_thread_.join();
  }

private:
  // Methods
  void sub_callback(const std_msgs::msg::Bool::SharedPtr ){
    new_msg_ = true;
    RCLCPP_INFO_STREAM(this->get_logger(), "Received new msg!");
    return;
  }

  void srv_callback(const std_srvs::srv::Trigger::Request::SharedPtr, std_srvs::srv::Trigger::Response::SharedPtr res){
    RCLCPP_INFO_STREAM(this->get_logger(), "Received request");
    new_msg_ = false;

    // Create subscription
    auto sub_options_ = rclcpp::SubscriptionOptions();
    sub_options_.callback_group = callback_group_;
    auto subscription = this->create_subscription<std_msgs::msg::Bool>(
          "topic_bool",
          rclcpp::QoS(1),
          std::bind(
            &CallbackGroupSrvServer::sub_callback,
            this,
            std::placeholders::_1),
          sub_options_
          );

    // Wait for new_msg
    while (rclcpp::ok()
           && new_msg_ == false) {
      rclcpp::sleep_for(std::chrono::seconds(1));
      RCLCPP_INFO_STREAM(this->get_logger(), "Sleep...");
    }

    res->message = "";
    res->success = true;
    return;
  }

  // Attributes
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr service_;
  rclcpp::CallbackGroup::SharedPtr callback_group_;
  rclcpp::executors::SingleThreadedExecutor callback_group_executor_;
  std::thread callback_group_executor_thread_;
  bool new_msg_;
};

int main(int argc, char * argv[]){
  rclcpp::init(argc, argv);
  auto srv_server = std::make_shared<CallbackGroupSrvServer>();
  rclcpp::spin(srv_server);
  rclcpp::shutdown();
  return 0;
}