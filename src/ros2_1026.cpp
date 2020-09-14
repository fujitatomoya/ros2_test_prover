#include <rclcpp/rclcpp.hpp>

class CheckTimerNode : public rclcpp::Node
{
public:
  CheckTimerNode(const rclcpp::NodeOptions& options) :
    Node("CheckTimerNode", options)
    {
      m_clock = std::make_shared< rclcpp::Clock >(RCL_SYSTEM_TIME);
      m_lastTime1 = m_clock->now().seconds();
      m_timer1 = this->create_wall_timer(
        std::chrono::milliseconds(1000),
        std::bind(&CheckTimerNode::timer1Callback, this));

      m_lastTime2 = m_clock->now().seconds();
      m_timer2 =this->create_wall_timer(
        std::chrono::milliseconds(200),
        std::bind(&CheckTimerNode::timer2Callback, this));
    }
private:
  void timer1Callback()
    {
      const auto newTime = m_clock->now().seconds();
      RCLCPP_WARN(this->get_logger(), "Timer 1 interval %f s", newTime - m_lastTime1);
      m_lastTime1 = newTime;
      std::this_thread::sleep_for(std::chrono::milliseconds(400));
    }

  void timer2Callback()
    {
      const auto newTime = m_clock->now().seconds();
      RCLCPP_WARN(this->get_logger(), "Timer 2 interval %f s", newTime - m_lastTime2);
      m_lastTime2 = newTime;
    }

  rclcpp::Clock::SharedPtr m_clock;
  double m_lastTime1;
  rclcpp::TimerBase::SharedPtr m_timer1;
  double m_lastTime2;
  rclcpp::TimerBase::SharedPtr m_timer2;
};

int main(int argc, char** argv)
{
  setvbuf(stdout, NULL, _IONBF, BUFSIZ);

  rclcpp::init(argc, argv);
  rclcpp::executors::SingleThreadedExecutor executor;
  //rclcpp::executors::MultiThreadedExecutor executor;
  rclcpp::NodeOptions options;
  auto node = std::make_shared< CheckTimerNode >(options);
  executor.add_node(node);
  executor.spin();
  rclcpp::shutdown();
  return 0;
}
