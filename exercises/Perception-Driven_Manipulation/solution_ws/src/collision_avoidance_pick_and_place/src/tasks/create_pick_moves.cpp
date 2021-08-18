#include <collision_avoidance_pick_and_place/pick_and_place.h>
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

std::vector<geometry_msgs::msg::Pose> collision_avoidance_pick_and_place::PickAndPlaceApp::create_pick_moves(
    geometry_msgs::msg::Pose &box_pose)
{
  //ROS_ERROR_STREAM("create_pick_moves is not implemented yet.  Aborting."); exit(1);

  // task variables
  tf2::Transform world_to_tcp_tf;
  tf2::Transform world_to_box_tf;
  tf2::Transform tcp_to_wrist_tf;
  geometry_msgs::msg::TransformStamped tcp_to_wrist_msg;
  std::vector<geometry_msgs::msg::Pose> tcp_pick_poses, wrist_pick_poses;


  /* Fill Code:
   * Goal:
   * - Create tcp pose at box pick.
   * Hints:
   * - Use the "setOrigin" to set the position of "world_to_tcp_tf".
   */
  //tf2::poseMsgToTF(box_pose,world_to_box_tf);
  world_to_box_tf = tf2::poseMsgToTransform(box_pose);
  tf2::Vector3 box_position(box_pose.position.x, box_pose.position.y, box_pose.position.z);
  world_to_tcp_tf.setOrigin(box_position);


  /* Setting tcp orientation
	   * Inverting the approach direction so that the tcp points towards the box instead of
	   * away from it.*/
  world_to_tcp_tf.setRotation(world_to_box_tf.getRotation()* tf2::Quaternion(tf2::Vector3(1,0,0),M_PI));


  // create all the poses for tcp's pick motion (approach, pick and retreat)
  tcp_pick_poses = create_manipulation_poses(cfg.RETREAT_DISTANCE, cfg.APPROACH_DISTANCE, world_to_tcp_tf);

  /* Fill Code:
   * Goal:
   * - Find transform from tcp to wrist.
   * Hints:
   * - Use the "lookupTransform" method of tf_buffer to get the transform
   */
  transform_buffer.waitForTransform(cfg.TCP_LINK_NAME, cfg.WRIST_LINK_NAME,
                                           rclcpp::Time(0),rclcpp::Duration::from_seconds(3.0),
                                           nullptr);

  tcp_to_wrist_msg  = transform_buffer.lookupTransform(cfg.TCP_LINK_NAME, cfg.WRIST_LINK_NAME,
                             rclcpp::Time(0), rclcpp::Duration::from_seconds(5.0));
  tf2::fromMsg(tcp_to_wrist_msg.transform,tcp_to_wrist_tf);

  /* Fill Code:
   * Goal:
   * - Transform list of pick poses from tcp frame to wrist frame
   * Hint:
   * - Use the "transform_from_tcp_to_wrist" function and save results into
   * 	"wrist_pick_poses".
   * - The "tcp_to_wrist_tf" is the transform that will help convert "tcp_pick_poses"
   * 	into "wrist_pick_poses".
   */
  wrist_pick_poses = transform_from_tcp_to_wrist(tcp_to_wrist_tf, tcp_pick_poses);

  // printing some results
  RCLCPP_INFO_STREAM(node->get_logger(), "tcp position at pick: " << "[" <<
                     world_to_tcp_tf.getOrigin().getX() <<
                     ", " << world_to_tcp_tf.getOrigin().getY()
                     << ", " << world_to_tcp_tf.getOrigin().getZ() << "]");
  RCLCPP_INFO_STREAM(node->get_logger(), "tcp z direction at pick: " << "[" << world_to_tcp_tf.getBasis().getColumn(2).getX() << ", " << world_to_tcp_tf.getBasis().getColumn(2).getY() << ", "  << world_to_tcp_tf.getBasis().getColumn(2).getZ() << "]");
  RCLCPP_INFO_STREAM(node->get_logger(), "wrist position at pick: " << wrist_pick_poses[1].position);

  return wrist_pick_poses;
}


