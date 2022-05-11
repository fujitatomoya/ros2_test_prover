#include <rclcpp/rclcpp.hpp>

#include <iostream>

class TestNode : public rclcpp::Node
{
    private:

	public:

		TestNode()
		: Node("test_node")
		{

		}
		virtual ~TestNode()
		{

		}
};


int main(int argc, char * argv[])
{
	std::cout << "Main!" << std::endl;
	rclcpp::init(argc, argv);
	auto node = std::make_shared<TestNode>();

	std::cout << "Starting node. ROS2 OK: " << rclcpp::ok() << "." << std::endl;
	rclcpp::spin(node);

	rclcpp::shutdown();

	return 0;
}
