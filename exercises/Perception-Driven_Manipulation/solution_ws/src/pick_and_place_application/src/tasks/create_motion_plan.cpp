#include <pick_and_place_application/pick_and_place.h>

/* CREATE MOTION PLAN
  Goal:
    - Creates a motion plan request using the desired end-effector pose and the current
        robot state (joint configuration and payload status)
    - Calls the moveit motion planning service and returns the motion plan if a valid one is
        found.

  Hints:

*/

namespace pick_and_place_application
{

bool PickAndPlaceApp::create_motion_plan(const geometry_msgs::msg::Pose &pose_target,
    const moveit_msgs::msg::RobotState &start_robot_state_msg,
    moveit_cpp::PlanningComponent::PlanSolution &plan_solution)
{
  // constructing motion plan goal constraints
  std::vector<double> position_tolerances(3,0.01f);
  std::vector<double> orientation_tolerances(3,0.01f);
  geometry_msgs::msg::PoseStamped p;
  p.header.frame_id = cfg.WORLD_FRAME_ID;
  p.pose = pose_target;
  moveit_msgs::msg::Constraints pose_goal = kinematic_constraints::constructGoalConstraints(
      cfg.WRIST_LINK_NAME,
      p,
      position_tolerances,
      orientation_tolerances);

  if(!moveit_cpp->getPlanningSceneMonitor()->requestPlanningSceneState())
  {
    throw std::runtime_error("Failed to get planning scene");
  }

  // setting up planning configuration
  moveit_cpp::PlanningComponent planning_component(cfg.ARM_GROUP_NAME, moveit_cpp);
  moveit_cpp::PlanningComponent::PlanRequestParameters plan_parameters;
  plan_parameters.planner_id = "RRTConnectkConfigDefault";
  plan_parameters.load(node);
  plan_parameters.planning_time = 20.0;
  plan_parameters.planning_attempts = 4;

  // set planning goal
  moveit::core::RobotState start_robot_state(moveit_cpp->getRobotModel());
  moveit::core::robotStateMsgToRobotState(start_robot_state_msg, start_robot_state);
  planning_component.setStartState(start_robot_state);
  planning_component.setGoal({pose_goal});

  // planning for goal
  plan_solution = planning_component.plan(plan_parameters);
  if(!plan_solution)
  {
    return false;
  }

  return true;
}

}




