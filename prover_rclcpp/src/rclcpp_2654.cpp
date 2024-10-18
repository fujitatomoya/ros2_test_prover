#include <chrono>
#include <functional>
#include <memory>

#include <rclcpp/rclcpp.hpp>

class DemoNode : public rclcpp::Node {
 public:
  DemoNode() : rclcpp::Node("demo127") {
    this->create_wall_timer(
      std::chrono::milliseconds(100),
      std::bind(&DemoNode::read, this));
    }
	
 private:
  void read() noexcept {
    // etc.
  }
};

int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<DemoNode>();
  rclcpp::spin(node);
  return rclcpp::shutdown();
}
