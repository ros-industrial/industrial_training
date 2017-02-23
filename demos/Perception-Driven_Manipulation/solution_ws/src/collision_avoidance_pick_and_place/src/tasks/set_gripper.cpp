#include <collision_avoidance_pick_and_place/pick_and_place.h>

/*    SET GRIPPER
  Goal:
	  - Turn the vacuum gripper on or off.
  Hints:
  	  - Use the grasp action client to send an grasp request to the grasp server.
  	  - Confirm that the gripper was successfully opened or closed and exit on error
*/
void collision_avoidance_pick_and_place::PickAndPlace::set_gripper(bool do_grasp)
{
  //ROS_ERROR_STREAM("set_gripper is not implemented yet.  Aborting."); exit(1);

  // task variables
  object_manipulation_msgs::GraspHandPostureExecutionGoal grasp_goal;
  bool success;

  // set the corresponding gripper action in the "grasp_goal" object.
  if (do_grasp)
    grasp_goal.goal = object_manipulation_msgs::GraspHandPostureExecutionGoal::GRASP;
  else
    grasp_goal.goal = object_manipulation_msgs::GraspHandPostureExecutionGoal::RELEASE;

  /* Fill Code:
   * Goal:
   * - Send the grasp goal to the server.
   * Hints:
   * - Use the 'sendGoal' method of the grasp client "grasp_action_client_ptr"
   * to make a call the server.
   */
  grasp_action_client_ptr->sendGoal(grasp_goal);


  /* Fill Code:
   * Goal:
   * - Confirm that client service call succeeded.
   * Hints:
   * - Use the "waitForResult" method of the client to wait for completion.
   * - Give "waitForResult" a timeout value of 4 seconds
   * - Timeouts in ros can be created using "ros::Duration(4.0f)".
   * - Save returned boolean from waitForResult() in the "success" variable.
   */
  success = grasp_action_client_ptr->waitForResult(ros::Duration(4.0f));

  if(success)
  {
    if (do_grasp)
      ROS_INFO_STREAM("Gripper closed");
    else
      ROS_INFO_STREAM("Gripper opened");
  }
  else
  {
    ROS_ERROR_STREAM("Gripper failure");
    exit(1);
  }
}


