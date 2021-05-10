#include <string>

#include <rclcpp/rclcpp.hpp>
#include <rosgraph_msgs/msg/clock.hpp>

class SimTimePublisher : public rclcpp::Node
{
  public:
    SimTimePublisher()
    : Node("sim_time_publisher")
    {
      clock_pub_ = this->create_publisher<rosgraph_msgs::msg::Clock>("/clock", 1);
      timer_ = this->create_wall_timer(
        std::chrono::milliseconds(1), // 1kHz constant
        std::bind(&SimTimePublisher::timer_callback, this));
    }

  private:
    void timer_callback()
    {
      rosgraph_msgs::msg::Clock clock;
      clock.clock = this->now();
      //RCLCPP_INFO(this->get_logger(), "Publishing: '%f'", rclcpp::Time(clock.clock).seconds());
      clock_pub_->publish(clock);
    }

    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<rosgraph_msgs::msg::Clock>::SharedPtr clock_pub_;
  };

  int main(int argc, char * argv[])
  {
    rclcpp::init(argc, argv);
    std::shared_ptr<SimTimePublisher> node = std::make_shared<SimTimePublisher>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
  }