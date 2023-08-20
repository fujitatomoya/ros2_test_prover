#include <string>

#include <rclcpp/rclcpp.hpp>
#include <rosgraph_msgs/msg/clock.hpp>

static int64_t timestart;

class SimTimePublisher : public rclcpp::Node
{
  public:
    SimTimePublisher()
    : Node("sim_time_publisher")
    {
      timestart = 0;
      clock_pub_ = this->create_publisher<rosgraph_msgs::msg::Clock>("/clock", 1);
      timer_ = this->create_wall_timer(
        std::chrono::milliseconds(100), // 10 Hz the same with gazebo publish rate
        std::bind(&SimTimePublisher::timer_callback, this));
    }

  private:
    void timer_callback()
    {
      rosgraph_msgs::msg::Clock clock;
      timestart += 100000000; // 100 msec with 10Hz
      rclcpp::Time time(timestart);
      clock.clock = time;
      //clock.clock = this->now(); // same with system clock
      RCLCPP_INFO(this->get_logger(), "Publishing: '%f'", rclcpp::Time(clock.clock).seconds());
      clock_pub_->publish(clock);
      //std::this_thread::sleep_for(std::chrono::milliseconds{50}); // sleep 50ms and republish the same time
      //clock_pub_->publish(clock);
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