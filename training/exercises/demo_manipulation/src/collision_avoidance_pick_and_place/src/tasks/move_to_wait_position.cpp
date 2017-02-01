#include <collision_avoidance_pick_and_place/pick_and_place.h>

/* MOVING ARM TO WAIT POSITION
  Goal:
    - Use the 'move_group' interface to move the robot to the 'wait' target.
    - Observe how we verify that the move was completed

  Hints:
    - 'cfg.WAIT_POSE_NAME' contains the name of the wait target.
    - Once the target is set you can call the 'move' method in order to go to that target.
*/

void collision_avoidance_pick_and_place::PickAndPlace::move_to_wait_position()
{
  //ROS_ERROR_STREAM("move_to_wait_position is not implemented yet.  Aborting."); exit(1);

  // task variables
  bool success; // saves the move result

  /* Fill Code:
   * Goal:
   * - Set robot wait target
   * Hints:
   * - Use the 'setNamedTarget' method in the 'move_group' object.
   * */
  move_group_ptr->setNamedTarget(cfg.WAIT_POSE_NAME);

  // set allowed planning time
  move_group_ptr->setPlanningTime(60.0f);

  /* Fill Code:
   * Goal:
   * - Move the robot
   * Hints:
   * - Use the 'move' method in the 'move_group' object and save the result
   *  in the 'success' variable*/
  success = move_group_ptr->move();
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


