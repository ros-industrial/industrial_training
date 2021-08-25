#include <pick_and_place_application/pick_and_place.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>


/*    CREATE PLACE MOVES
  Goal:
    - Set the pose of the tcp at the box place pose
    - Create tcp poses for the place motion (approach, release, retreat).
    - Find transform of the wrist in tcp coordinates
    - Convert tcp pick poses to wrist poses.
         * MoveIt's kinematics require the target position to be specified relative to
           one of the kinematic links of the manipulator arm (as defined in the SRDF)

  Hints:
    - You can manipulate the "world_to_tcp_tf" transform through the "setOrigin" and "setRotation".
    - Use the "create_manipulation_poses" function to create the tcp poses between each place move
    - Use the "transform_from_tcp_to_wrist" function to populate the "wrist_place_poses" array.
*/

std::vector<geometry_msgs::msg::Pose> pick_and_place_application::PickAndPlaceApp::create_place_moves()
{
  //ROS_ERROR_STREAM("create_place_moves is not implemented yet.  Aborting."); exit(1);

  // task variables
  tf2::Transform world_to_tcp_tf, tcp_to_wrist_tf;
  geometry_msgs::msg::TransformStamped tcp_to_wrist_msg;
  std::vector<geometry_msgs::msg::Pose> tcp_place_poses, wrist_place_poses;


  /* Fill Code:
   * Objective:
   * - Find the desired tcp pose at box place
   * Hints:
   * - Use the "setOrigin" method to set the position of "world_to_tcp_tf"
   * 	using cfg.BOX_PLACE_TF.
   * - cfg.BOX_PLACE_TF is a tf::Transform object so it provides a getOrigin() method.
   */
  world_to_tcp_tf.setOrigin(cfg.BOX_PLACE_TF.getOrigin());

  /* Fill Code:
   * Goal:
   * - Reorient the tool so that the tcp points towards the box.
   * Hints:
   * - Use the "setRotation" to set the orientation of "world_to_tcp_tf".
   * - The quaternion value "tf::Quaternion(0.707, 0.707, 0, 0)" will point
   * 	the tcp's direction towards the box.
   */
  world_to_tcp_tf.setRotation(tf2::Quaternion(0.707, 0.707, 0, 0));


  /* Fill Code:
   * Goal:
   * - Create place poses for tcp.   *
   * Hints:
   * - Use the "create_manipulation_poses" and save results to "tcp_place_poses".
   * - Look in the "cfg" object to find the corresponding retreat and approach distance
   * 	values.
   */
 tcp_place_poses = create_manipulation_poses(cfg.RETREAT_DISTANCE, cfg.APPROACH_DISTANCE, world_to_tcp_tf);


  /* Fill Code:
   * Goal:
   * - Find transform from tcp to wrist.
   * Hints:
   * - Use the "lookupTransform" method in the transform listener.
   * */
  transform_buffer.waitForTransform(cfg.TCP_LINK_NAME, cfg.WRIST_LINK_NAME, node->get_clock()->now(),
                             rclcpp::Duration::from_seconds(3.0f), [](const tf2_ros::TransformStampedFuture& ){});
  tcp_to_wrist_msg = transform_buffer.lookupTransform(cfg.TCP_LINK_NAME, cfg.WRIST_LINK_NAME, node->get_clock()->now(),
                                              rclcpp::Duration::from_seconds(5.0) );
  tf2::fromMsg(tcp_to_wrist_msg.transform, tcp_to_wrist_tf);


  /* Fill Code:
   * Goal:
   * - Transform list of place poses from the tcp to the wrist coordinate frame.
   * Hints:
   * - Use the "transform_from_tcp_to_wrist" function and save results into
   * 	"wrist_place_poses".
   * - The "tcp_to_wrist_tf" is the transform that will help convert "tcp_place_poses"
   * 	into "wrist_place_poses".
   */
  wrist_place_poses = transform_from_tcp_to_wrist(tcp_to_wrist_tf, tcp_place_poses);

  // printing results
  RCLCPP_INFO_STREAM(node->get_logger(), "tcp position at place: " << "[" << world_to_tcp_tf.getOrigin().getX() << ", " << world_to_tcp_tf.getOrigin().getY() << ", " << world_to_tcp_tf.getOrigin().getZ() << "]");
  RCLCPP_INFO_STREAM(node->get_logger(), "wrist position at place: " << wrist_place_poses[1].position);

  return wrist_place_poses;
}

