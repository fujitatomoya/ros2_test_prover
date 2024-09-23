#include "example_interfaces/action/fibonacci.hpp"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

using Fibonacci = example_interfaces::action::Fibonacci;
using GoalHandle = rclcpp_action::ServerGoalHandle<Fibonacci>;

class Server final : public rclcpp::Node {
public:
  Server() : rclcpp::Node("action_server") {
    using namespace std::placeholders;

    m_cb = create_callback_group(rclcpp::CallbackGroupType::Reentrant);
    m_server = rclcpp_action::create_server<Fibonacci>(
      this,
      "/test/server",
      std::bind(&Server::goalHandle, this, _1, _2),
      std::bind(&Server::handleCancel, this, _1),
      std::bind(&Server::handleAccepted, this, _1),
      rcl_action_server_get_default_options(),
      m_cb
      );
  }

  Server(Server const &) = delete;
  Server(Server &&) noexcept = delete;
  Server& operator=(Server const &) = delete;
  Server& operator=(Server &&) noexcept = delete;
  ~Server() noexcept = default;

  rclcpp::CallbackGroup::SharedPtr m_cb;
  rclcpp_action::Server<Fibonacci>::SharedPtr m_server;

private:
  rclcpp_action::GoalResponse goalHandle(rclcpp_action::GoalUUID const &uuid,
                                         Fibonacci::Goal::ConstSharedPtr goal) {
    (void)uuid;
    RCLCPP_INFO_STREAM(get_logger(), "Goal handle");
    if (goal->order > 100) {
      return rclcpp_action::GoalResponse::REJECT;
    }
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }
    
  rclcpp_action::CancelResponse handleCancel(std::shared_ptr<GoalHandle> const goalHandle) {
    RCLCPP_INFO_STREAM(get_logger(), "Handle cancel");
    (void)goalHandle;
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void handleAccepted(std::shared_ptr<GoalHandle> const goalHandle) {
    RCLCPP_INFO_STREAM(get_logger(), "Handle accepted");
    std::thread([goal = goalHandle, this] () -> void {
                  this->execute(std::move(goal));
                }).detach();
  }

  void execute(std::shared_ptr<GoalHandle> const goalHandle) {
    RCLCPP_INFO(this->get_logger(), "Executing goal");
    rclcpp::Rate loop_rate(1);
    const auto goal = goalHandle->get_goal();
    auto feedback = std::make_shared<Fibonacci::Feedback>();
    auto & sequence = feedback->sequence;
    sequence.push_back(0);
    sequence.push_back(1);
    auto result = std::make_shared<Fibonacci::Result>();

    for (int i = 1; (i < goal->order) && rclcpp::ok(); ++i) {
      // Check if there is a cancel request
      if (goalHandle->is_canceling()) {
        result->sequence = sequence;
        goalHandle->canceled(result);
        RCLCPP_INFO(this->get_logger(), "Goal canceled");
        return;
      }
      // Update sequence
      sequence.push_back(sequence[i] + sequence[i - 1]);
      // Publish feedback
      goalHandle->publish_feedback(feedback);
      RCLCPP_INFO(this->get_logger(), "Publish feedback");

      loop_rate.sleep();
    }

    if (rclcpp::ok()) {
      result->sequence = sequence;
      goalHandle->succeed(result);
      RCLCPP_INFO(this->get_logger(), "Goal succeeded");
    }
  }
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);

  auto const server = std::make_shared<Server>();

  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(server);
  executor.spin();

  rclcpp::shutdown();
};
