#include <pick_and_place_application/pick_and_place.h>

/*    SET GRIPPER
  Goal:
    - Turn the vacuum gripper on or off.
  Hints:
    - Use the grasp action client to send an grasp request to the grasp server.
    - Confirm that the gripper was successfully opened or closed and exit on error
*/
void pick_and_place_application::PickAndPlaceApp::actuateGripper(bool do_grasp)
{
  using GraspGoalType = pick_and_place_msgs::action::ExecuteGraspMove::Goal;
  using GraspGoalHandle = rclcpp_action::ClientGoalHandle<pick_and_place_msgs::action::ExecuteGraspMove>;
  // RCLCPP_ERROR_STREAM(node,"set_gripper is not implemented yet.  Aborting."); exit(1);

  // task variables
  GraspGoalType grasp_goal;

  // set the corresponding gripper action in the "grasp_goal" object.
  if (do_grasp)
    grasp_goal.goal = GraspGoalType::GRASP;
  else
    grasp_goal.goal = GraspGoalType::RELEASE;

  /* Fill Code:
   * Goal:
   * - Send the grasp goal to the server.
   * Hints:
   * - Use the "sendGoal" method of the grasp client "grasp_action_client_ptr"
   * to make a call to the server.
   */
  std::shared_future<GraspGoalHandle::SharedPtr> goal_fut = grasp_action_client->async_send_goal(grasp_goal);
  std::future_status st = goal_fut.wait_for(std::chrono::duration<double>(2));

  /* Fill Code:
   * Goal:
   * - Confirm that client service call succeeded.
   * Hints:
   * - Use the "waitForResult" method of the client to wait for completion.
   * - Give "waitForResult" a timeout value of 4 seconds
   * - Timeouts in ros can be created using "ros::Duration(4.0f)".
   * - Save returned boolean from waitForResult() in the "success" variable.
   */
  // success = grasp_action_client_ptr->waitForResult(ros::Duration(4.0f));

  if (st == std::future_status::ready)
  {
    if (do_grasp)
      RCLCPP_INFO_STREAM(node->get_logger(), "Gripper closed");
    else
      RCLCPP_INFO_STREAM(node->get_logger(), "Gripper opened");
  }
  else
  {
    RCLCPP_ERROR_STREAM(node->get_logger(), "Gripper failure");
    exit(1);
  }
}
