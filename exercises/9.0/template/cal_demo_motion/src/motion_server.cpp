#include <rclcpp/rclcpp.hpp>
#include <moveit/moveit_cpp/moveit_cpp.h>
#include <moveit/moveit_cpp/planning_component.h>
#include <cal_demo_motion/srv/move_robot.hpp>
#include <tf2_eigen/tf2_eigen.hpp>

class AETPlanner : public rclcpp::Node
{
public:

    AETPlanner(std::string name) : Node(name, rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true))
    {
        server_ = this->create_service<cal_demo_motion::srv::MoveRobot>("move_robot", 
            std::bind(&AETPlanner::plan_callback, this, std::placeholders::_1, std::placeholders::_2));

    }

    void setup_moveit()
    {
        moveit_cpp_ = std::make_shared<moveit_cpp::MoveItCpp>(this->shared_from_this());

        // Parameters set on this node
        plan_parameters_.load(this->shared_from_this());
    }

    void plan_callback(const std::shared_ptr<cal_demo_motion::srv::MoveRobot::Request> request,
                        std::shared_ptr<cal_demo_motion::srv::MoveRobot::Response> response)
    {
        RCLCPP_INFO(this->get_logger(), "Got planning request");

        // getting current state of robot from environment
        if (!moveit_cpp_->getPlanningSceneMonitor()->requestPlanningSceneState())
        {
            std::string error_msg = "Failed to get planning scene";
            RCLCPP_ERROR(this->get_logger(), error_msg.c_str());
            response->success = false;
            response->message = error_msg;
            return;
        }

        moveit::core::RobotStatePtr current_robot_state = moveit_cpp_->getCurrentState(2.0);

        // Planning component associated with a single motion group
        moveit_cpp::PlanningComponent planning_component = moveit_cpp::PlanningComponent("arm", moveit_cpp_);

        planning_component.setStartState(*current_robot_state);
        planning_component.setGoal(request->goal, "tool0");

        // Now we can plan!
        moveit_cpp::PlanningComponent::PlanSolution plan_solution = planning_component.plan(plan_parameters_);
        if (!plan_solution)
        {
            std::string error_msg = "Planning failed";
            RCLCPP_ERROR(this->get_logger(), error_msg.c_str());
            response->success = false;
            response->message = error_msg;
            return;
        }

        if (!moveit_cpp_->execute("arm", plan_solution.trajectory, true))
        {
          std::string error_msg = "Failed to execute trajectory";
          RCLCPP_ERROR(this->get_logger(), error_msg.c_str());
          response->success = false;
          response->message = error_msg;
          return;
        }

        response->success = true;
        response->message = "Motion successful";
        RCLCPP_INFO(this->get_logger(), "Success!");

        moveit_msgs::msg::RobotTrajectory moveit_trajectory;
        plan_solution.trajectory->getRobotTrajectoryMsg(moveit_trajectory);
        response->trajectory = moveit_trajectory.joint_trajectory;

        return;
    }

private:

    rclcpp::Service<cal_demo_motion::srv::MoveRobot>::SharedPtr server_;
    moveit_cpp::MoveItCppPtr moveit_cpp_;
    moveit_cpp::PlanningComponent::PlanRequestParameters plan_parameters_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);

  std::shared_ptr<AETPlanner> planner = std::make_shared<AETPlanner>("planning_server");
  planner->setup_moveit();

  RCLCPP_INFO(planner->get_logger(), "Ready to plan");

  rclcpp::spin(planner);
  rclcpp::shutdown();
}