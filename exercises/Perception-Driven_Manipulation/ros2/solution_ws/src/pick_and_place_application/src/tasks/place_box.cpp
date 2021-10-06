#include <pick_and_place_application/pick_and_place.h>

/* MOVE ARM THROUGH PLACE POSES
  Goal:
    - Move the robot through the place locations.
    - Open gripper after reaching the release location.
  Hints:
    - Use the methods seen so far such as "move", "sendGoal", "waitForResult" whenever needed.
*/

void pick_and_place_application::PickAndPlaceApp::doBoxPlace(std::vector<geometry_msgs::msg::Pose>& place_poses,
                                                             const geometry_msgs::msg::Pose& box_pose)
{
  // RCLCPP_ERROR_STREAM(node->get_logger(),"doBoxPlace is not implemented yet.  Aborting."); exit(1);

  // task variables
  bool success;

  // move the robot to each wrist place pose
  for (unsigned int i = 0; i < place_poses.size(); i++)
  {
    moveit::core::RobotStatePtr robot_state = moveit_cpp->getCurrentState(2.0);
    if (!robot_state)
    {
      RCLCPP_ERROR_STREAM(node->get_logger(), "Failed to get robot state");
      throw std::runtime_error("Failed to place box");
    }
    moveit_msgs::msg::RobotState robot_state_msg;
    moveit::core::robotStateToRobotStateMsg(*robot_state, robot_state_msg, true);

    if (i == 0)
    {
      // attaching box
      setAttachedObject(true, box_pose, robot_state_msg);
      showBox(true);
    }
    else if (i == 1)
    {
      // detaching box prior to place so that the place pose is not in collision
      setAttachedObject(false, box_pose, robot_state_msg);
      showBox(true);
    }
    else
    {
      // detaching box
      setAttachedObject(false, geometry_msgs::msg::Pose(), robot_state_msg);
      showBox(false);
    }

    // create motion plan
    moveit_cpp::PlanningComponent::PlanSolution plan_solution;
    if(!doMotionPlanning(place_poses[i], robot_state_msg, plan_solution))
    {
      throw std::runtime_error("Failed to plan trajectory to place location");
    }

    /* Fill Code:
     * Goal:
     * - Execute the planned trajectory
     * Hints:
     * - Use the "moveit_cpp->execute(...)" method to execute the trajectory on the robot
     */
    success = false;
    success = moveit_cpp->execute(cfg.ARM_GROUP_NAME, plan_solution.trajectory, true);
    if (success)
    {
      RCLCPP_INFO_STREAM(node->get_logger(), "Place Move " << i << " Succeeded");
    }
    else
    {
      RCLCPP_ERROR_STREAM(node->get_logger(), "Place Move " << i << " Failed");
      actuateGripper(false);
      throw std::runtime_error("Failed to place box");
    }

    if (i == 1)
    {
      /* Fill Code:
       * Goal:
       * - Turn off gripper suction after the release pose is reached.
       * Hints:
       * - Call the "set_gripper" function to turn on suction.
       * - The input to the set_gripper method takes a "true" or "false"
       *       boolean argument.
       */
      actuateGripper(false);
    }
  }
}
