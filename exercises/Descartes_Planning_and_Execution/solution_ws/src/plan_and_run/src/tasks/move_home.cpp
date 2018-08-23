#include <plan_and_run/demo_application.h>

/* MOVE HOME
  Goal:
    - Use the moveit MoveGroup interface to move the arm to a pre-recorded positions saved in the moveit config package.
    - Verify that the arm reached the target.

  Hints:
    - Call the "move_group_interface::MoveGroupInterface::move()" method to move the arm.
    - The "result.val" is an integer flag which indicates either success or an error condition.
*/

namespace plan_and_run
{

void DemoApplication::moveHome()
{
  //ROS_ERROR_STREAM("Task '"<<__FUNCTION__ <<"' is incomplete. Exiting"); exit(-1);

  // creating move group interface for planning simple moves
  moveit::planning_interface::MoveGroupInterface move_group(config_.group_name);
  move_group.setPlannerId(PLANNER_ID);

  // setting home position as target
  if(!move_group.setNamedTarget(HOME_POSITION_NAME))
  {
    ROS_ERROR_STREAM("Failed to set home '"<<HOME_POSITION_NAME<<"' position");
    exit(-1);
  }

  // moving home
  /*  Fill Code:
   * Goal:
   * - Call the move_group.move() and save the returned flag into the "result" variable
   * - Verify that the robot reached the goal.
   * Hint:
   * - The "result.val" and "result.SUCCESS" flags can be used to verify that the move was completed
   * -
   */
  moveit_msgs::MoveItErrorCodes result = move_group.move();
  if(result.val != result.SUCCESS)
  {
    ROS_ERROR_STREAM("Failed to move to "<<HOME_POSITION_NAME<<" position");
    exit(-1);
  }
  else
  {
    ROS_INFO_STREAM("Robot reached home position");
  }

  ROS_INFO_STREAM("Task '"<<__FUNCTION__<<"' completed");

}

}

