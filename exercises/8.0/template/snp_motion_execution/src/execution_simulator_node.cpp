#include <std_srvs/srv/trigger.hpp>
#include <memory>
#include <functional>
#include <thread>
#include <control_msgs/action/follow_joint_trajectory.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

class ExecSimServer : public rclcpp::Node
{
public:
  explicit ExecSimServer(const std::string& name, const rclcpp::NodeOptions& options = rclcpp::NodeOptions())
    : Node(name, options)
  {
    using namespace std::placeholders;

    this->action_server_ = rclcpp_action::create_server<control_msgs::action::FollowJointTrajectory>(
        this, "joint_trajectory_action", std::bind(&ExecSimServer::handleGoal, this, _1, _2),
        std::bind(&ExecSimServer::handleCancel, this, _1), std::bind(&ExecSimServer::handleAccepted, this, _1));

    this->service_srv_ = this->create_service<std_srvs::srv::Trigger>(
        "robot_enable", std::bind(&ExecSimServer::set_response, this, _1, _2));

    RCLCPP_INFO(this->get_logger(), "Node started");
  }

private:
  rclcpp_action::Server<control_msgs::action::FollowJointTrajectory>::SharedPtr action_server_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr service_srv_;

  rclcpp_action::GoalResponse

  handleGoal(const rclcpp_action::GoalUUID& /*uuid*/,
             std::shared_ptr<const control_msgs::action::FollowJointTrajectory::Goal> /*goal*/)
  {
    RCLCPP_INFO(this->get_logger(), "Received goal request");
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse
  handleCancel(const std::shared_ptr<
               rclcpp_action::ServerGoalHandle<control_msgs::action::FollowJointTrajectory>> /*goal_handle*/)
  {
    RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");
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
    RCLCPP_INFO(this->get_logger(), "Executing goal");
    auto result = std::make_shared<control_msgs::action::FollowJointTrajectory::Result>();
    result->error_code = control_msgs::action::FollowJointTrajectory::Result::SUCCESSFUL;
    goal_handle->succeed(result);
    RCLCPP_INFO(this->get_logger(), "Goal succeeded");
    return;
  }

  void set_response(const std::shared_ptr<std_srvs::srv::Trigger::Request> /*request*/,
                    std::shared_ptr<std_srvs::srv::Trigger::Response> response)
  {
    response->success = true;
    RCLCPP_INFO(this->get_logger(), "Robot Enabled");
  }
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<ExecSimServer>("exec_sim_server");

  rclcpp::spin(node);
  rclcpp::shutdown();
}
