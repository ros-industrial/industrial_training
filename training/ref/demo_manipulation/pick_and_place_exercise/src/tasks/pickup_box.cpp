/*
 * pickup_box.cpp
 *
 *  Created on: Jan 10, 2014
 *      Author: ros-industrial
 */


#include <pick_and_place_exercise/pick_and_place.h>

/* MOVE ARM THROUGH PICK POSES
  Goal:
    - Use the 'move_group' object to set the wrist as the end-effector link
    - Use the 'move_group' object to set the world frame as the reference frame for path planning
    - Move the robot to each pick pose.
    - Close gripper after reaching the approach pose

  Hints:
    - The 'move_group' interface has useful methods such as 'setEndEffectorLink' and 'setPoseReferenceFrame' that
      can be used to prepare the robot for planning.
    - The 'setPoseTarget' method allows you to set a "pose" as your target to move the robot.
*/
void PickAndPlace::pickup_box(std::vector<geometry_msgs::Pose>& pick_poses,const geometry_msgs::Pose& box_pose)
{
	  //ROS_ERROR_STREAM("move_through_pick_poses is not implemented yet.  Aborting."); exit(1);

	  // task variables
	  object_manipulation_msgs::GraspHandPostureExecutionGoal grasp_goal;
	  bool success;

	  // set the wrist as the end-effector link
	  //   - the robot will try to move this link to the specified position
	  //   - if not specified, MoveIt will use the last link in the arm group
	  /* Fill Code: [ use the 'setEndEffectorLink' in the 'move_group' object] */
	  move_group_ptr->setEndEffectorLink(cfg.WRIST_LINK_NAME);

	  // set allowed planning time
	  move_group_ptr->setPlanningTime(60.0f);

	  // set world frame as the reference
	  //   - the target position is specified relative to this frame
	  //   - if not specified, MoveIt will use the parent frame of the SRDF "Virtual Joint"
	  /* Fill Code: [ use the 'setPoseReferenceFrame' in the 'move_group' object] */
	  move_group_ptr->setPoseReferenceFrame(cfg.WORLD_FRAME_ID);

	  // move the robot to each wrist pick pose
	  for(unsigned int i = 0; i < pick_poses.size(); i++)
	  {

	    // set the current pose as the target
	    /* Fill Code: [ use the 'setPoseTarget' method in the 'move_group' object and pass the current pose in 'pick_poses'] */
		move_group_ptr->setPoseTarget(pick_poses[i]);

	    // moving arm to current pick pose
	    /* Fill Code: [ use the 'move' method in the 'move_group' object and save the result in the 'success' variable] */
	    success = move_group_ptr->move();

	    // verifying move completion
	    if(success)
	    {
	      ROS_INFO_STREAM("Pick Move " << i <<" Succeeded");
	    }
	    else
	    {
	      ROS_ERROR_STREAM("Pick Move " << i <<" Failed");
	      exit(1);
	    }


	    if(i == 0)
	    {
		// turn on gripper suction after approach pose
		/* Fill Code: [ call the 'set_gripper' function to turn on suction ] */
	      set_gripper(true);

	    }


	    if(i == 2)
	    {
	    	// attach box to end effector
	    	set_attached_object(true,box_pose);
	    }

	  }

}

