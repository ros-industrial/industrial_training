#include <pick_and_place_application/pick_and_place.h>

/* MOVE ARM THROUGH PLACE POSES
  Goal:
    - Move the robot through the entire place motion.
    - Open gripper after reaching the release pose.
  Hints:
    - Use the methods seen so far such as "move", "sendGoal", "waitForResult" whenever needed.
*/

void pick_and_place_application::PickAndPlaceApp::place_box(std::vector<geometry_msgs::msg::Pose>& place_poses,
        const geometry_msgs::msg::Pose& box_pose)
{
  //RCLCPP_ERROR_STREAM(node,"place_box is not implemented yet.  Aborting."); exit(1);

  // task variables
  bool success;

  /* Fill Code:
   * Goal:
   * - Set the ReferenceFrame and EndEffectorLink
   * Hints:
   * - Use the "setEndEffectorLink" and "setPoseReferenceFrame" methods of "move_group_ptr"
   */
  //moveit_cpp_ptr->setEndEffectorLink(cfg.WRIST_LINK_NAME);
  //moveit_cpp_ptr->setPoseReferenceFrame(cfg.WORLD_FRAME_ID);

  // set allowed planning time
  //moveit_cpp_ptr->setPlanningTime(60.0f);

  // move the robot to each wrist place pose
  for(unsigned int i = 0; i < place_poses.size(); i++)
  {
    moveit::core::RobotStatePtr robot_state = moveit_cpp->getCurrentState(2.0);
    if(!robot_state)
    {
      RCLCPP_ERROR_STREAM(node->get_logger(),"Failed to get robot state");
      throw std::runtime_error("Failed to place box");
    }
    moveit_msgs::msg::RobotState robot_state_msg;
    moveit::core::robotStateToRobotStateMsg(*robot_state, robot_state_msg, true);

    if(i==0 || i == 1)
    {
      // attaching box
      set_attached_object(true,box_pose,robot_state_msg);
      show_box(true);

    }
    else
    {
      // detaching box
      set_attached_object(false,geometry_msgs::msg::Pose(),robot_state_msg);
      show_box(false);
    }

    // create motion plan
    moveit_cpp::PlanningComponent::PlanSolution plan_solution;
    success = create_motion_plan(place_poses[i], robot_state_msg,plan_solution)
        && moveit_cpp->execute(cfg.ARM_GROUP_NAME, plan_solution.trajectory, true);

    if(success)
    {
      RCLCPP_INFO_STREAM(node->get_logger(),"Place Move " << i <<" Succeeded");
    }
    else
    {
      RCLCPP_ERROR_STREAM(node->get_logger(),"Place Move " << i <<" Failed");
      set_gripper(false);
      throw std::runtime_error("Failed to place box");
    }


    if(i == 1)
    {
      /* Fill Code:
       * Goal:
       * - Turn off gripper suction after the release pose is reached.
       * Hints:
       * - Call the "set_gripper" function to turn on suction.
       * - The input to the set_gripper method takes a "true" or "false"
       *       boolean argument.
       */
      set_gripper(false);
    }

  }
}



