// Created by ubuntu on 23-11-28.
//
#include <chrono>
#include <functional>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/bool.hpp"
#include <std_msgs/msg/string.hpp>

using namespace std::chrono_literals;
using std::placeholders::_1;

class ChangeWorkFrame: public rclcpp::Node
{
  public:
    ChangeWorkFrame();                                                                   
    void change_work_frame_programe();                                                 
  
  private:

    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;                 

    size_t count_;
};


void ChangeWorkFrame::change_work_frame_programe()
{
    std_msgs::msg::String change_work_frame;
    change_work_frame.data = "sss" + std::to_string(this->count_++);                          
    RCLCPP_INFO(this->get_logger(), " Publishing '%s'", change_work_frame.data.c_str());
    this->publisher_->publish(change_work_frame);
}
/***********************************************end**************************************************/


ChangeWorkFrame::ChangeWorkFrame():rclcpp::Node("changeframe")
{
  publisher_ = this->create_publisher<std_msgs::msg::String>("topic", 10);
  sleep(2);
  change_work_frame_programe();
}
/***********************************************end**************************************************/

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ChangeWorkFrame>());
  rclcpp::shutdown();
  return 0;
}