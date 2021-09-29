#include <pick_and_place_application/pick_and_place.h>

/* CREATE MOTION PLAN
  Goal:
    - Learn how to use the moveit_cpp::PlanningComponent class to plan a trajectory that moves the robot from
       the current robot state (joint configuration and payload status) to a desired pose of the tcp.
    - Understand how to use the kinematic_constraints::constructGoalConstraints to create a goal cartesian pose for the tcp.

*/

namespace pick_and_place_application
{
bool PickAndPlaceApp::doMotionPlanning(const geometry_msgs::msg::Pose& pose_target,
                                       const moveit_msgs::msg::RobotState& start_robot_state_msg,
                                       moveit_cpp::PlanningComponent::PlanSolution& plan_solution)
{
  // constructing motion plan goal constraints
  std::vector<double> position_tolerances(3, 0.01f);
  std::vector<double> orientation_tolerances(3, 0.01f);
  geometry_msgs::msg::PoseStamped goal_pose;
  goal_pose.header.frame_id = cfg.WORLD_FRAME_ID;
  goal_pose.pose = pose_target;
  moveit_msgs::msg::Constraints robot_goal = kinematic_constraints::constructGoalConstraints(
      cfg.TCP_LINK_NAME, goal_pose, position_tolerances, orientation_tolerances);

  if (!moveit_cpp->getPlanningSceneMonitor()->requestPlanningSceneState())
  {
    throw std::runtime_error("Failed to get planning scene");
  }

  // setting up planning configuration
  moveit_cpp::PlanningComponent planning_component(cfg.ARM_GROUP_NAME, moveit_cpp);
  moveit_cpp::PlanningComponent::PlanRequestParameters plan_parameters;
  plan_parameters.load(node);
  plan_parameters.planner_id = cfg.PLANNER_ID;
  plan_parameters.planning_time = cfg.PLANNING_TIME;
  plan_parameters.planning_attempts = cfg.PLANNING_ATTEMPTS;

  // set planning goal
  moveit::core::RobotState start_robot_state(moveit_cpp->getRobotModel());
  moveit::core::robotStateMsgToRobotState(start_robot_state_msg, start_robot_state);
  planning_component.setStartState(start_robot_state);
  planning_component.setGoal({ robot_goal });

  // planning for goal
  plan_solution = planning_component.plan(plan_parameters);
  if (!plan_solution)
  {
    return false;
  }

  return true;
}

}  // namespace pick_and_place_application
