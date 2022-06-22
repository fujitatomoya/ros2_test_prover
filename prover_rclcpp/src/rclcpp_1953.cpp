#include <iostream>
#include <thread>
#include <string>

#include <rclcpp/node.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/wait_for_message.hpp>
#include <std_msgs/msg/string.hpp>

using namespace std::chrono_literals;


/// Get current node state in a loop
void wait_for_messages(std::shared_ptr<rclcpp::Node> node) {
	// infinite loop to receive the messages
   	while (rclcpp::ok()) {
		std_msgs::msg::String out;
		auto ret = rclcpp::wait_for_message(out, node, "chatter", 1s);
        if (ret) {
            RCLCPP_INFO(node->get_logger(), "WaitForMessage: [%s]", out.data.c_str());
		} else { // timeout or error
			RCLCPP_INFO(node->get_logger(), "WaitForMessage: cannot receive the message");
		}
   	}
}


class TestNode : public rclcpp::Node
{
public:

	TestNode()
	: Node("test_node")
	{
		auto callback = [this](const std_msgs::msg::String::SharedPtr msg) -> void
          {
            RCLCPP_INFO(this->get_logger(), "Subscription: [%s]", msg->data.c_str());
          };
        sub_ = this->create_subscription<std_msgs::msg::String>("chatter", 10, callback);
	}

	virtual ~TestNode()
	{
		// empty
	}

private:
	rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_;
};


int main(int argc, char * argv[])
{
	// initialize node and create subscription on it
	rclcpp::init(argc, argv);
	auto node = std::make_shared<TestNode>();

	// create another thread to issue `wait_for_message` with TestNode object
	std::thread thread_object(wait_for_messages, node);

	// give it 5 sec to make sure `wait_for_message` works
	std::this_thread::sleep_for(std::chrono::seconds(5));

	// spin the node
	rclcpp::spin(node);
	rclcpp::shutdown();

	return 0;
}
