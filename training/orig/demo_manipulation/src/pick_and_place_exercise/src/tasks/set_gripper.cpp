/*
 * open_gripper.cpp
 *
 *  Created on: Jun 3, 2013
 *      Author: ros-industrial
 */

#include <pick_and_place_exercise/pick_and_place.h>

/*    SET GRIPPER
  Goal:
    - Use the grasp action client to open or close the gripper.
    - Confirm that the gripper was successfully opened or closed.  Exit program on failure;
  Hints:
*/
void set_gripper(GraspActionClient& grasp_action_client, bool do_grasp)
{
  ROS_ERROR_STREAM("set_gripper is not implemented yet.  Aborting."); exit(1);

  // task variables
  object_manipulation_msgs::GraspHandPostureExecutionGoal grasp_goal;
  bool success;

  // send grasp goal to open gripper
  /* Fill Code: [ set the "suction off" goal using the correct GraspHandPostureExecutionGoal constant ] */
  if (do_grasp)
    grasp_goal.goal = object_manipulation_msgs::GraspHandPostureExecutionGoal::GRASP;
  else
    grasp_goal.goal = 0;  // REPLACE THIS VALUE

  /* Fill Code: [ use the 'sendGoal' method of the grasp client to open gripper] */


  // confirm that gripper opened
  /* Fill Code: [ use the 'waitForResult' to check and save result in success variable] */
  /*   - can you specify a timeout for the wait command? */


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


