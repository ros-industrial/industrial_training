#include <collision_avoidance_pick_and_place/pick_and_place.h>

/* CREATE PICK MOVES
  Goal:
    - Set the pose for the tcp at the box pick.
    - Create tcp poses for the pick motions (approach, target, retreat).
    - Find transform of the wrist in tcp coordinates
    - Convert tcp pick poses to wrist poses.
         * MoveIt's kinematics require the target position to be specified relative to
           one of the kinematic links of the manipulator arm (as defined in the SRDF)

  Hints:
    - You can manipulate the 'world_to_tcp_tf' transform through the 'setOrigin' and 'setRotation'.
    - Look into the 'create_manipulation_poses' function and observe how each pick pose is created.
    - Use the 'transform_from_tcp_to_wrist' function to populate the 'wrist_pick_poses' array.
*/

std::vector<geometry_msgs::Pose> collision_avoidance_pick_and_place::PickAndPlace::create_pick_moves(geometry_msgs::Pose &box_pose)
{
  //ROS_ERROR_STREAM("create_pick_moves is not implemented yet.  Aborting."); exit(1);

  // task variables
  tf::Transform world_to_tcp_tf;
  tf::Transform world_to_box_tf;
  tf::StampedTransform tcp_to_wrist_tf;
  std::vector<geometry_msgs::Pose> tcp_pick_poses, wrist_pick_poses;


  /* Fill Code:
   * Goal:
   * - Create tcp pose at box pick.
   * Hints:
   * - Use the 'setOrigin' to set the position of 'world_to_tcp_tf'.
   * */
  tf::poseMsgToTF(box_pose,world_to_box_tf);
  tf::Vector3 box_position(box_pose.position.x, box_pose.position.y, box_pose.position.z);
  world_to_tcp_tf.setOrigin(box_position);

  // setting tcp orientation
  /* Inverting the approach direction so that the tcp points towards the box instead of
   * away from it.*/
  world_to_tcp_tf.setRotation(world_to_box_tf.getRotation()* tf::Quaternion(tf::Vector3(1,0,0),M_PI));


  // create all the poses for tcp's pick motion (approach, pick and retreat)
  tcp_pick_poses = create_manipulation_poses(cfg.RETREAT_DISTANCE, cfg.APPROACH_DISTANCE, world_to_tcp_tf);


  /* Fill Code:
   * Goal:
   * - Find transform from tcp to wrist.
   * Hints:
   * - Use the 'lookupTransform' method in the transform listener.
   * */
  transform_listener_ptr->waitForTransform(cfg.TCP_LINK_NAME, cfg.WRIST_LINK_NAME,ros::Time::now(),ros::Duration(3.0f));
  transform_listener_ptr->lookupTransform(cfg.TCP_LINK_NAME, cfg.WRIST_LINK_NAME, ros::Time(0.0f), tcp_to_wrist_tf);


  /* Fill Code:
   * Goal:
   * - Transform list of pick poses from tcp frame to wrist frame
   * Hint:
   * - Use the 'transform_from_tcp_to_wrist' function and save results into
   * 	'wrist_pick_poses'.
   * 	*/
  wrist_pick_poses = transform_from_tcp_to_wrist(tcp_to_wrist_tf, tcp_pick_poses);

  // printing some results
  ROS_INFO_STREAM("tcp position at pick: " << world_to_tcp_tf.getOrigin());
  ROS_INFO_STREAM("tcp z direction at pick: " << world_to_tcp_tf.getBasis().getColumn(2));
  ROS_INFO_STREAM("wrist position at pick: " << wrist_pick_poses[1].position);

  return wrist_pick_poses;
}


