#include <memory>
#include <functional>
#include <thread>
#include <control_msgs/action/follow_joint_trajectory.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

template <typename T>
T declare_and_get(rclcpp::Node* node, const std::string& key)
{
  T val;
  node->declare_parameter(key);
  if (!node->get_parameter(key, val))
    throw std::runtime_error("Failed to get '" + key + "' parameter");
  return val;
}

class ExecSimServer : public rclcpp::Node
{
public:
  explicit ExecSimServer(const std::string& name, const rclcpp::NodeOptions& options = rclcpp::NodeOptions())
    : Node(name, options)
  {
    using namespace std::placeholders;

    auto fjt_action = declare_and_get<std::string>(this, "follow_joint_trajectory_action");
    action_server_ = rclcpp_action::create_server<control_msgs::action::FollowJointTrajectory>(
        this, fjt_action, std::bind(&ExecSimServer::handleGoal, this, _1, _2),
        std::bind(&ExecSimServer::handleCancel, this, _1), std::bind(&ExecSimServer::handleAccepted, this, _1));

    RCLCPP_INFO(get_logger(), "Started simulated robot execution node");
  }

private:
  rclcpp_action::Server<control_msgs::action::FollowJointTrajectory>::SharedPtr action_server_;

  rclcpp_action::GoalResponse
  handleGoal(const rclcpp_action::GoalUUID& /*uuid*/,
             std::shared_ptr<const control_msgs::action::FollowJointTrajectory::Goal> /*goal*/)
  {
    RCLCPP_INFO(get_logger(), "Received goal request");
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse
  handleCancel(const std::shared_ptr<
               rclcpp_action::ServerGoalHandle<control_msgs::action::FollowJointTrajectory>> /*goal_handle*/)
  {
    RCLCPP_INFO(get_logger(), "Received request to cancel goal");
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void handleAccepted(
      const std::shared_ptr<rclcpp_action::ServerGoalHandle<control_msgs::action::FollowJointTrajectory>> goal_handle)
  {
    using namespace std::placeholders;

    std::thread{ std::bind(&ExecSimServer::execute, this, _1), goal_handle }.detach();
  }

  void execute(
      const std::shared_ptr<rclcpp_action::ServerGoalHandle<control_msgs::action::FollowJointTrajectory>> goal_handle)
  {
    RCLCPP_INFO(get_logger(), "Executing goal");
    auto result = std::make_shared<control_msgs::action::FollowJointTrajectory::Result>();
    result->error_code = control_msgs::action::FollowJointTrajectory::Result::SUCCESSFUL;
    goal_handle->succeed(result);
    RCLCPP_INFO(get_logger(), "Goal succeeded");
    return;
  }
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<ExecSimServer>("motion_execution_server_sim");

  rclcpp::spin(node);
  rclcpp::shutdown();
}
