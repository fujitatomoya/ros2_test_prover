#include <chrono>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/int32.hpp>
#include <std_srvs/srv/trigger.hpp>

using namespace std::chrono_literals;

class IDoRosStuff
{
  public:
    IDoRosStuff(const rclcpp::Node::SharedPtr& node) : n(node), i(0)
    {
    }

    void init()
    {
        pub_a = n->create_publisher<std_msgs::msg::Int32>("~/a", rclcpp::SystemDefaultsQoS());
        pub_b = n->create_publisher<std_msgs::msg::Int32>("~/b", rclcpp::SystemDefaultsQoS());

        srv_a = n->create_service<std_srvs::srv::Trigger>("~/a", std::bind(&IDoRosStuff::serviceACb, this, std::placeholders::_1, std::placeholders::_2));
        srv_b = n->create_service<std_srvs::srv::Trigger>("~/b", std::bind(&IDoRosStuff::serviceBCb, this, std::placeholders::_1, std::placeholders::_2));
    }

    void run()
    {
        std_msgs::msg::Int32 msg;
        msg.data = i;
        pub_a->publish(msg);
        pub_b->publish(msg);
        i++;
    }

    rclcpp::Parameter param_a, param_b, param_c;
    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr pub_a, pub_b;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr srv_a, srv_b;
    rclcpp::Node::SharedPtr n;
    unsigned int i;

    void serviceACb(const std::shared_ptr<std_srvs::srv::Trigger::Request> req, std::shared_ptr<std_srvs::srv::Trigger::Response> resp)
    {
        (void)req;
        (void)resp;
        param_a = n->get_parameter("a");
        std::cout << "param_a: " << param_a << std::endl;
    }

    void serviceBCb(const std::shared_ptr<std_srvs::srv::Trigger::Request> req, std::shared_ptr<std_srvs::srv::Trigger::Response> resp)
    {
        (void)req;
        (void)resp;
        param_b = n->get_parameter("b");
        std::cout << "param_b: " << param_b << std::endl;
    }
};

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::NodeOptions options;
    options.allow_undeclared_parameters(true);
    options.automatically_declare_parameters_from_overrides(true);
    rclcpp::Node::SharedPtr node = rclcpp::Node::make_shared("top_level", "top_level", options);

    std::vector<std::string> sub_node_names = { "one", "two", "three", "four", "five", "six", "seven", "eight", "nine", "ten" };
    std::vector<std::shared_ptr<IDoRosStuff>> my_classes;
    double iter = 0.;
    for (const std::string& name : sub_node_names)
    {
        // create sub nodes
        rclcpp::Node::SharedPtr n = node->create_sub_node(name);
        // set sub nodes parameters
        n->set_parameter({ "a", 0. * iter });
        n->set_parameter({ "b", 1. * iter });
        n->set_parameter({ "c", 2. * iter });
        iter += 1.;

        std::shared_ptr<IDoRosStuff> a_class(new IDoRosStuff(n));
        a_class->init();
        my_classes.push_back(a_class);
    }

    rclcpp::WallRate loop_rate(500ms);

    auto executor = rclcpp::executors::SingleThreadedExecutor();
    executor.add_node(node);
    while (rclcpp::ok())
    {
        try
        {
            for (std::shared_ptr<IDoRosStuff> ptr : my_classes)
            {
                ptr->run();
            }

            executor.spin_some();
        }
        catch (const rclcpp::exceptions::RCLError& e)
        {
            RCLCPP_ERROR(node->get_logger(), "unexpectedly failed with %s", e.what());
        }
        loop_rate.sleep();
    }
    rclcpp::shutdown();
    return 0;
}