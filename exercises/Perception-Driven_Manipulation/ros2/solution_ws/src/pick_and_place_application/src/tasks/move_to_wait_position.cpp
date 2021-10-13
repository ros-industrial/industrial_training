#include <pick_and_place_application/pick_and_place.h>

using moveit::planning_interface::MoveItErrorCode;

/* MOVING ARM TO WAIT POSITION
  Goal:
    - Use the "moveit_cpp::PlanningComponent" to plan a trajectory that moves the robot to the "wait" target.
    - Execute the planned trajectory on the robot

  Hints:
    - "cfg.WAIT_POSE_NAME" contains the name of the wait target.
    - You can send a trajectory with the "execute()" method of the "moveit_cpp" object.
*/

void pick_and_place_application::PickAndPlaceApp::moveToWaitPosition()
{
  // RCLCPP_ERROR(node->get_logger(),"moveToWaitPosition is not implemented yet.  Aborting."); exit(1);

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

  /*

     Fill Code:
     * Goal:
     * - Set robot wait target
     * Hints:
     * - Use the "setGoal" method in the "planning_component" object.
     * - The "WAIT_POSE_NAME" variable is a member of the "cfg" object.
     */
  planning_component.setGoal(cfg.WAIT_POSE_NAME);

  // now plan the trajectory
  moveit_cpp::PlanningComponent::PlanSolution plan_solution = planning_component.plan(plan_parameters);
  if (plan_solution)
  {
    RCLCPP_INFO_STREAM(node->get_logger(), "Move " << cfg.WAIT_POSE_NAME << " Succeeded");
  }
  else
  {
    RCLCPP_INFO_STREAM(node->get_logger(), "Move " << cfg.WAIT_POSE_NAME << " Failed");
    throw std::runtime_error("Failed to plan move to wait position");
  }

  /*
   Fill Code:
   * Goal:
   * - Execute the trajectory on the robot
   * Hints:
   * - Use the "execute" method in the "moveit_cpp" object and save the result
   *  in the "succeeded" variable
   */

  bool succeeded = false;
  succeeded = moveit_cpp->execute(cfg.ARM_GROUP_NAME, plan_solution.trajectory, true);

  // check if the execution succeeded
  if (!succeeded)
  {
    throw std::runtime_error("Failed to execute trajectory to wait position");
  }
}
