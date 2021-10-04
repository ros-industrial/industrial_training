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
  // RCLCPP_ERROR_STREAM(node->get_logger(),"set_gripper is not implemented yet.  Aborting."); exit(1);

  // task variables
  GraspGoalType grasp_goal;

  // set the corresponding gripper action in the "grasp_goal" object.
  if (do_grasp)
    grasp_goal.goal = GraspGoalType::GRASP;
  else
    grasp_goal.goal = GraspGoalType::RELEASE;

  /* Fill Code:
   * Goal:
   * - Send the grasp goal to the action server.
   * Hints:
   * - Use the "async_send_goal" method of the "grasp_action_client"
   * to send the goal to the server.
   */
  std::shared_future<GraspGoalHandle::SharedPtr> goal_fut = grasp_action_client->async_send_goal(grasp_goal);


  /* Fill Code:
   * Goal:
   * - Confirm that a valid result was sent by the action server.
   * Hints:
   * - Use the "wait_for" method of the goal_fut future object to wait for completion.
   * - Give "wait_for" a timeout value of 4 seconds
   * - The "wait_for" method takes an argument of the "std::chrono::duration<double>(4)" type.
   * - Save the output of wait for in the status variable
   */
  std::future_status status = std::future_status::deferred;
  status = goal_fut.wait_for(std::chrono::duration<double>(4));

  if (status == std::future_status::ready)
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
