/*
 * task_move_to_wait_position.cpp
 *
 *  Created on: Jun 3, 2013
 *      Author: ros-industrial
 */

#include <pick_and_place_exercise/pick_and_place.h>

/* MOVING ARM TO WAIT POSITION
  Goal:
    - Use the 'move_group' interface to move the robot to the 'wait' target.
    - Observe how we verify that the move was completed

  Hints:
    - 'cfg.WAIT_POSE_NAME' contains the name of the wait target.
    - Once the target is set you can call the 'move' method in order to go to that target.
*/

void move_to_wait_position(move_group_interface::MoveGroup& move_group)
{
  ROS_ERROR_STREAM("move_to_wait_position is not implemented yet.  Aborting."); exit(1);

  // task variables
  bool success; // saves the move result

  // set robot wait target
  /* Fill Code: [ use the 'setNamedTarget' method in the 'move_group' object] */


  // move the robot
  /* Fill Code: [ use the 'move' method in the 'move_group' object and save the result in the 'success' variable] */

  if(success)
  {
    ROS_INFO_STREAM("Move " << cfg.WAIT_POSE_NAME<< " Succeeded");
  }
  else
  {
    ROS_ERROR_STREAM("Move " << cfg.WAIT_POSE_NAME<< " Failed");
    exit(1);
  }
}


