#include <pick_and_place_application/pick_and_place.h>

using moveit::planning_interface::MoveItErrorCode;

/* MOVING ARM TO WAIT POSITION
  Goal:
    - Use the "move_group" interface to move the robot to the "wait" target.
    - Observe how we verify that the move was completed

  Hints:
    - "cfg.WAIT_POSE_NAME" contains the name of the wait target.
    - Once the target is set you can call the "move" method in order to go to that target.
*/

void pick_and_place_application::PickAndPlaceApp::moveToWaitPosition()
{
  //ROS_ERROR_STREAM("move_to_wait_position is not implemented yet.  Aborting."); exit(1);

  if(!moveit_cpp->getPlanningSceneMonitor()->requestPlanningSceneState())
  {
    throw std::runtime_error("Failed to get planning scene");
  }

/*  // task variables
  MoveItErrorCode error; // saves the move result

   Fill Code:
   * Goal:
   * - Set robot wait target
   * Hints:
   * - Use the "setNamedTarget" method in the "move_group_ptr" object.
   * - Look in the "cfg.WAIT_POSE_NAME" object for the name of the target.
   */

  // setting up planning configuration
  moveit_cpp::PlanningComponent planning_component(cfg.ARM_GROUP_NAME, moveit_cpp);
  moveit_cpp::PlanningComponent::PlanRequestParameters plan_parameters;
  plan_parameters.planner_id = "RRTConnectkConfigDefault";
  plan_parameters.load(node);
  plan_parameters.planning_time = 20.0f;
  plan_parameters.planning_attempts = 4;

  planning_component.setGoal(cfg.WAIT_POSE_NAME);

  /*
   Fill Code:
   * Goal:
   * - Move the robot
   * Hints:
   * - Use the "move" method in the "move_group_ptr" object and save the result
   *  in the "error" variable
   */

  moveit_cpp::PlanningComponent::PlanSolution plan_solution = planning_component.plan();
  if(plan_solution)
  {
    RCLCPP_INFO_STREAM(node->get_logger(), "Move " << cfg.WAIT_POSE_NAME<< " Succeeded");
  }
  else
  {
    RCLCPP_INFO_STREAM(node->get_logger(),"Move " << cfg.WAIT_POSE_NAME<< " Failed");
    throw std::runtime_error("Failed to plan move to wait position");
  }

  if(!moveit_cpp->execute(cfg.ARM_GROUP_NAME, plan_solution.trajectory, true))
  {
    throw std::runtime_error("Failed to execute trajectory to wait position");
  }
}


