/*
 * move_through_place_poses.cpp
 *
 *  Created on: Jun 3, 2013
 *      Author: ros-industrial
 */

#include <pick_and_place_exercise/pick_and_place.h>

/* MOVE ARM THROUGH PLACE POSES
  Goal:
    - Move the robot to each place pose.
    - Open gripper after reaching the target pose
  Hints:
    - Use the methods seen so far such as 'move', 'sendGoal', 'waitForResult' as needed
*/

void PickAndPlace::place_box(std::vector<geometry_msgs::Pose>& place_poses)
{
  //ROS_ERROR_STREAM("move_through_place_poses is not implemented yet.  Aborting."); exit(1);

  // task variables
  object_manipulation_msgs::GraspHandPostureExecutionGoal grasp_goal;
  bool success;

  // set the referenceFrame and EndEffectorLink
  /* Fill Code: [ use the 'setEndEffectorLink' and 'setPoseReferenceFrame' methods of 'move_group'] */
  move_group_ptr->setEndEffectorLink(cfg.WRIST_LINK_NAME);
  move_group_ptr->setPoseReferenceFrame(cfg.WORLD_FRAME_ID);

  // set allowed planning time
  move_group_ptr->setPlanningTime(60.0f);

  // move the robot to each wrist place pose
  for(unsigned int i = 0; i < place_poses.size(); i++)
  {
    // set the current place pose as the target
    /* Fill Code: [ use the 'setPoseTarget' method and pass the current pose in 'place_poses'] */
    move_group_ptr->setPoseTarget(place_poses[i]);

    // move arm to current place pose
    /* Fill Code: [ call the 'move' method to execute the move ] */
    success = move_group_ptr->move();

    if(success)
    {
      ROS_INFO_STREAM("Place Move " << i <<" Succeeded");
    }
    else
    {
      ROS_ERROR_STREAM("Place Move " << i <<" Failed");
      exit(1);
    }


    if(i ==0)
    {
      // detaching box
      set_attached_object(false);
    }

    if(i == 1)
    {
	// turn off gripper suction after reaching target pose
	/* Fill Code: [ call the 'set_gripper' function with the appropriate arguments ] */
	/*   - only call this once, after the target position has been reached */
	/*   - HINT: this should be the second pose in the sequence of place_poses */
      set_gripper(false);
    }

  }
}



