/*
 * move_through_place_poses.cpp
 *
 *  Created on: Jun 3, 2013
 *      Author: ros-industrial
 */

#include <collision_avoidance_pick_and_place/pick_and_place.h>

/* MOVE ARM THROUGH PLACE POSES
  Goal:
    - Move the robot to each place pose.
    - Open gripper after reaching the target pose
  Hints:
    - Use the methods seen so far such as 'move', 'sendGoal', 'waitForResult' as needed
*/

void collision_avoidance_pick_and_place::PickAndPlace::place_box(std::vector<geometry_msgs::Pose>& place_poses,
		const geometry_msgs::Pose& box_pose)
{
  //ROS_ERROR_STREAM("move_through_place_poses is not implemented yet.  Aborting."); exit(1);

  // task variables
  bool success;

  // set the referenceFrame and EndEffectorLink
  /* Fill Code: [ use the 'setEndEffectorLink' and 'setPoseReferenceFrame' methods of 'move_group'] */
  move_group_ptr->setEndEffectorLink(cfg.WRIST_LINK_NAME);
  move_group_ptr->setPoseReferenceFrame(cfg.WORLD_FRAME_ID);

  // move the robot to each wrist place pose
  for(unsigned int i = 0; i < place_poses.size(); i++)
  {
  	// create motion plan
    move_group_interface::MoveGroup::Plan plan;
    success = create_motion_plan(place_poses[i],plan) && move_group_ptr->execute(plan);

    if(success)
    {
      ROS_INFO_STREAM("Place Move " << i <<" Succeeded");
    }
    else
    {
      ROS_ERROR_STREAM("Place Move " << i <<" Failed");
      exit(1);
    }


    if(i == 1)
    {
	// turn off gripper suction after reaching target pose
	/* Fill Code: [ call the 'set_gripper' function with the appropriate arguments ] */
	/*   - only call this once, after the target position has been reached */
	/*   - HINT: this should be the second pose in the sequence of place_poses */
      set_gripper(false);
    }

  	if(i==0 )
  	{
      // attaching box
      set_attached_object(false);
  	}

  }
}



