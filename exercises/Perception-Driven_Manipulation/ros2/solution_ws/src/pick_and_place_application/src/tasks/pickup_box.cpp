#include <pick_and_place_application/pick_and_place.h>

/* MOVE ARM THROUGH PICK POSES
  Goal:
    - Get the current robot state to plan a trajectory to the pick position
    - Attach the box to the TCP
    - Execute the trajectory

  Hints:
    - The "moveit::core::robotStateToRobotStateMsg" function converts the robot_state object into a message
        and "setPoseReferenceFrame" that can be used to prepare the robot for planning.
*/

void pick_and_place_application::PickAndPlaceApp::doBoxPickup(std::vector<geometry_msgs::msg::Pose>& pick_poses,
                                                              const geometry_msgs::msg::Pose& box_pose)
{
  // RCLCPP_ERROR_STREAM(node->get_logger(),"doBoxPickup is not implemented yet.  Aborting."); exit(1);

  // task variables
  bool success;


  // move the robot to each wrist pick pose
  for (unsigned int i = 0; i < pick_poses.size(); i++)
  {
    /* Fill Code:
     * Goal:
     * - Get the current robot state before planning for a move
     * - Check that the robot state is valid.
     * Hints:
     * - Use the "moveit_cpp->getCurrentState(...)" method to get the current state from the environment.
     * - getCurrentState takes a timeout input parameter.  A timeout of ~2 seconds is probably fine.
     */
    moveit::core::RobotStatePtr robot_state = nullptr;
    robot_state = moveit_cpp->getCurrentState(2.0);
    if (!robot_state)
    {
      RCLCPP_ERROR_STREAM(node->get_logger(), "Failed to get robot state");
      exit(1);
    }
    moveit_msgs::msg::RobotState robot_state_msg;
    moveit::core::robotStateToRobotStateMsg(*robot_state, robot_state_msg, true);

    /* Inspect Code:
     * Goal:
     * - Look in the "setAttachedObject()" method to understand how to attach/detach a payload using MoveIt.
     */
    setAttachedObject(false, geometry_msgs::msg::Pose(), robot_state_msg);

    /* Inspect Code:
     * Goal:
     * - Look in the "doMotionPlanning()" method to observe how an
     * 	  entire MoveIt motion plan is produced.
     */
    moveit_cpp::PlanningComponent::PlanSolution plan_solution;
    if(!doMotionPlanning(pick_poses[i], robot_state_msg, plan_solution))
    {
      throw std::runtime_error("Failed to plan trajectory to pick location");
    }

    /* Fill Code:
     * Goal:
     * - Execute the planned trajectory
     * Hints:
     * - Use the "moveit_cpp->execute(...)" method to execute the trajectory on the robot
     * - You'll need to pass the arm group name and trajectory
     */
    success = false;
    success = moveit_cpp->execute(cfg.ARM_GROUP_NAME, plan_solution.trajectory, true);

    // verifying move completion
    if (success)
    {
      RCLCPP_INFO_STREAM(node->get_logger(), "Pick Move " << i << " Succeeded");
    }
    else
    {
      RCLCPP_ERROR_STREAM(node->get_logger(), "Pick Move " << i << " Failed");
      actuateGripper(false);
      throw std::runtime_error("Failed to pick up box");
    }

    if (i == 0)
    {
      /* Fill Code:
       * Goal:
       * - Turn on gripper suction after the approach position is reached.
       * Hints:
       * - Call the "actuateGripper" function to turn on suction.
       * - The input to the actuateGripper method takes a "true" or "false"
       *   boolean argument.
       */
      actuateGripper(true);
    }
  }
}
