#include <chrono>
#include <iostream>
#include <memory>
#include <thread>

#include "lifecycle_msgs/msg/transition.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/publisher.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "rcutils/logging_macros.h"

/// Get current node state in a loop
void checkState(std::shared_ptr<rclcpp_lifecycle::LifecycleNode> node) {

   // infinite loop
   while (1) {
      // This call fails
      node->get_current_state().id();
   }
}

int main(int argc, char * argv[])
{
   setvbuf(stdout, NULL, _IONBF, BUFSIZ);

   rclcpp::init(argc, argv);

   std::shared_ptr<rclcpp_lifecycle::LifecycleNode> lc_node =
           std::make_shared<rclcpp_lifecycle::LifecycleNode>("lc_talker");

   lc_node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);

   // Created multiple threads
   std::thread thread_object_1(checkState, lc_node);
   std::thread thread_object_2(checkState, lc_node);

   // wait
   thread_object_1.join();

   rclcpp::shutdown();

   return 0;
}