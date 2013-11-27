/*
 * create_pick_moves.cpp
 *
 *  Created on: Jun 3, 2013
 *      Author: ros-industrial
 */

#include <pick_and_place_exercise/pick_and_place.h>

/* CREATE PICK MOVES
  Goal:
    - Set the pose for the tcp at the box pick.
    - Create tcp poses to be used before/after pick moves (approach, target, retreat).
    - Find transform of the wrist in tcp coordinates
    - Convert tcp pick poses to wrist poses.
         * MoveIt's kinematics require the target position to be specified relative to
           one of the kinematic links of the manipulator arm (as defined in the SRDF)

  Hints:
    - You can manipulate the 'world_to_tcp_tf' transform through the 'setOrigin' and 'setRotation'.
    - Look into the 'create_manipulation_poses' function and observe how each pick pose is created.
    - Use the 'transform_from_tcp_to_wrist' function to populate the 'wrist_pick_poses' array.
*/
std::vector<geometry_msgs::Pose> create_pick_moves(tf::TransformListener &tf_listener, geometry_msgs::Pose &box_pose)
{
  ROS_ERROR_STREAM("create_pick_moves is not implemented yet.  Aborting."); exit(1);

  // task variables
  tf::Transform world_to_tcp_tf;
  tf::StampedTransform tcp_to_wrist_tf;
  std::vector<geometry_msgs::Pose> tcp_pick_poses, wrist_pick_poses;

  // create tcp pose at box pick
  //   - we manually specify the box-height, as this is difficult to determine from the AR tag
  //   - the orientation is set to point the end-effector "down" towards the box
  /* Fill Code: [ use the 'setOrigin' method to set the position of 'world_to_tcp_tf'] */
  tf::Vector3 box_position(box_pose.position.x, box_pose.position.y, cfg.BOX_SIZE.getZ());

  world_to_tcp_tf.setRotation(tf::Quaternion(M_PI, 0, M_PI/2.0f));

  // create pick poses for tcp
  tcp_pick_poses = create_manipulation_poses(cfg.RETREAT_DISTANCE, cfg.APPROACH_DISTANCE, world_to_tcp_tf);

  // find transform from tcp to wrist (in TCP frame)
  /* Fill Code: [ use the 'lookupTransform' method in the transform listener] */

  // transform list of pick positions from TCP frame to wrist frame
  /* Fill Code: [ use the 'transform_from_tcp_to_wrist' function and save results into 'wrist_pick_poses'] */

  // printing some results
  ROS_INFO_STREAM("tcp position at pick: " << world_to_tcp_tf.getOrigin());
  ROS_INFO_STREAM("tcp z direction at pick: " << world_to_tcp_tf.getBasis().getColumn(2));
  ROS_INFO_STREAM("wrist position at pick: " << wrist_pick_poses[1].position);

  return wrist_pick_poses;
}


