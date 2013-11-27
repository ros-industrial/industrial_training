/*
 * create_place_moves.cpp
 *
 *  Created on: Jun 3, 2013
 *      Author: ros-industrial
 */

#include <pick_and_place_exercise/pick_and_place.h>

/*    CREATE PLACE MOVES
  Goal:
    - Set the pose of the tcp at the box place pose
    - Create tcp poses to be used before/after place moves (approach, target, retreat).
    - Find transform of the wrist in tcp coordinates
    - Convert tcp pick poses to wrist poses.
         * MoveIt's kinematics require the target position to be specified relative to
           one of the kinematic links of the manipulator arm (as defined in the SRDF)

  Hints:
    - You can manipulate the 'world_to_tcp_tf' transform through the 'setOrigin' and 'setRotation'.
    - Use the 'create_manipulation_poses' function to create the tcp poses between each place move
    - Use the 'transform_from_tcp_to_wrist' function to populate the 'wrist_place_poses' array.
*/

std::vector<geometry_msgs::Pose> create_place_moves(tf::TransformListener& tf_listener)
{
  ROS_ERROR_STREAM("create_place_moves is not implemented yet.  Aborting."); exit(1);

  // task variables
  tf::Transform world_to_tcp_tf;
  tf::StampedTransform tcp_to_wrist_tf;
  std::vector<geometry_msgs::Pose> tcp_place_poses, wrist_place_poses;

  // finding tcp pose at box place
  /* Fill Code: [ use the 'setOrigin' method to set the position of 'world_to_tcp_tf' using cfg.BOX_PLACE_TF] */
  /*   - BOX_PLACE_TF is a tf::Transform, which has a getOrigin() method */
  /* Fill Code: [ use the 'setRotation' to set the orientation of 'world_to_tcp_tf'] */
  /*   - use the same "pointing down" orientation as in create_pick_moves() */


  // create place poses for tcp
  /* Fill Code: [ use the 'create_manipulation_poses' and save results to 'tcp_place_poses'] */


  // find transform from tcp to wrist (in TCP frame)
  /* Fill Code: [ use the 'lookupTransform' method in the transform listener] */


  // transform list of pick positions from TCP frame to wrist frame
  /* Fill Code: [ use the 'transform_from_tcp_to_wrist' function and save results into 'wrist_place_poses'] */


  // printing results
  ROS_INFO_STREAM("tcp position at place: " << world_to_tcp_tf.getOrigin());
  ROS_INFO_STREAM("wrist position at place: "<<wrist_place_poses[1].position);

  return wrist_place_poses;
}

