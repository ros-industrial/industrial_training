#include <pick_and_place_application/pick_and_place.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

/*    CREATE PLACE MOVES
  Goal:
    - Set the pose of the tcp at the box place pose
    - Create tcp poses for the place locations (approach, release, retreat).

  Hints:
    - You can manipulate the "world_to_tcp_tf" transform through the "setOrigin" and "setRotation".
    - Use the "createManipulationPoses" function to create the tcp poses between each place move
*/

std::vector<geometry_msgs::msg::Pose> pick_and_place_application::PickAndPlaceApp::computePlaceToolPoses()
{
  // RCLCPP_ERROR_STREAM(node->get_logger(), "computePlaceToolPoses is not implemented yet.  Aborting."); exit(1);

  // task variables
  tf2::Transform tcp_at_box_tf, tcp_to_wrist_tf;
  geometry_msgs::msg::TransformStamped tcp_to_wrist_msg;
  std::vector<geometry_msgs::msg::Pose> tcp_place_poses, wrist_place_poses;

  /* Fill Code:
   * Objective:
   * - Compute the TCP pose at the box place location
   * Hints:
   * - Use the "setOrigin" method to set the position of "tcp_at_box_tf"
   * 	using cfg.BOX_PLACE_TF.
   * - cfg.BOX_PLACE_TF is a tf2::Transform object so it provides a getOrigin() method.
   */
  tcp_at_box_tf.setOrigin(cfg.BOX_PLACE_TF.getOrigin());

  /* Fill Code:
   * Goal:
   * - Reorient the tool so that the tcp points towards the box.
   * Hints:
   * - Use the "setRotation" to set the orientation of "tcp_at_box_tf".
   * - The quaternion value "tf2::Quaternion(0.707, 0.707, 0, 0)" will point
   * 	the tcp's direction towards the box.
   */
  tcp_at_box_tf.setRotation(cfg.BOX_PLACE_TF.getRotation() * tf2::Quaternion(0.707, 0.707, 0, 0));

  /* Fill Code:
   * Goal:
   * - Create place poses for tcp.   *
   * Hints:
   * - Use the "createManipulationPoses" and save results to "tcp_place_poses".
   * - The RETREAT_DISTANCE and APPROACH_DISTANCE values were populated from a configuration yaml file passed to the executable in the launch file.
   */
  tcp_place_poses = createManipulationPoses(cfg.RETREAT_DISTANCE, cfg.APPROACH_DISTANCE, tcp_at_box_tf);

  // printing results
  RCLCPP_INFO_STREAM(node->get_logger(),
                     "tcp position at place: "
                         << "[" << tcp_at_box_tf.getOrigin().getX() << ", " << tcp_at_box_tf.getOrigin().getY() << ", "
                         << tcp_at_box_tf.getOrigin().getZ() << "]");

  return tcp_place_poses;
}
