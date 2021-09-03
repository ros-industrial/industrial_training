#include <pick_and_place_application/pick_and_place.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>


/* CREATE PICK MOVES
  Goal:
    - Set the pose for the tcp at the box pick.
    - Create tcp poses for the pick motions (approach, target, retreat).
    - Find transform of the wrist in tcp coordinates
    - Convert tcp pick poses to wrist poses.
         * Moveit's kinematics require the target position to be specified relative to
           one of the kinematic links of the manipulator arm (as defined in the SRDF)

  Hints:
    - You can manipulate the "world_to_tcp_tf" transform through the "setOrigin" and "setRotation".
    - Look into the "create_manipulation_poses" function and observe how each pick pose is created.
    - Use the "transform_from_tcp_to_wrist" function to populate the "wrist_pick_poses" array.
*/

std::vector<geometry_msgs::msg::Pose> pick_and_place_application::PickAndPlaceApp::computePickToolPoses(
    geometry_msgs::msg::Pose &box_pose)
{
  //ROS_ERROR_STREAM("create_pick_moves is not implemented yet.  Aborting."); exit(1);

  // transforms relative to world
  tf2::Transform tcp_at_box_tf; // transform of tcp at box relative to world
  tf2::Transform box_tf; // transform of box at box relative to world
  std::vector<geometry_msgs::msg::Pose> tcp_pick_poses;

  /* Fill Code:
   * Goal:
   * - Create tcp pose at box pick.
   * Hints:
   * - Use the "setOrigin" to set the position of "world_to_tcp_tf".
   */
  //tf2::poseMsgToTF(box_pose,world_to_box_tf);
  box_tf = tf2::poseMsgToTransform(box_pose);
  tf2::Vector3 box_position(box_pose.position.x, box_pose.position.y, box_pose.position.z);
  tcp_at_box_tf.setOrigin(box_position);

  /* Setting tcp orientation
	   * Inverting the approach direction so that the tcp points towards the box instead of
	   * away from it.*/
  tcp_at_box_tf.setRotation(box_tf.getRotation()* tf2::Quaternion(tf2::Vector3(1,0,0),M_PI));

  // create all the poses for tcp's pick motion (approach, pick and retreat)
  tcp_pick_poses = createManipulationPoses(cfg.RETREAT_DISTANCE, cfg.APPROACH_DISTANCE, tcp_at_box_tf);

  // printing some results
  RCLCPP_INFO_STREAM(node->get_logger(), "tcp position at pick: " << "[" <<
		  tcp_at_box_tf.getOrigin().getX() <<", " << tcp_at_box_tf.getOrigin().getY()
                     << ", " << tcp_at_box_tf.getOrigin().getZ() << "]");
  RCLCPP_INFO_STREAM(node->get_logger(), "tcp z direction at pick: " << "[" << tcp_at_box_tf.getBasis().getColumn(2).getX() <<
		  ", " << tcp_at_box_tf.getBasis().getColumn(2).getY() << ", "  << tcp_at_box_tf.getBasis().getColumn(2).getZ() << "]");


  return tcp_pick_poses;
}


