/*
 * detect_box_pick.cpp
 *
 *  Created on: Jun 3, 2013
 *      Author: ros-industrial
 */

#include <pick_and_place_exercise/pick_and_place.h>

/* DETECTING BOX PICK POSE
  Goal:
    - Find the box's position in the world frame using the transform listener.
        * this transform is published by the kinect AR-tag perception node
    - Save the pose into 'box_pose'.

  Hints:
    - lookupTransform can also look "in the past".  Use Time=0 to get the most-recent transform.
    - tf::poseTFToMsg allows converting transforms into Pose messages
*/
geometry_msgs::Pose detect_box_pick(tf::TransformListener &tf_listener)
{
  //ROS_ERROR_STREAM("detect_box_pick is not implemented yet.  Aborting."); exit(1);

  // task variables
  tf::StampedTransform world_to_box_pick_tf;
  geometry_msgs::Pose box_pose;

  // use transform listener to find the box's pick pose (relative to world frame)
  /* Fill Code: [ use the 'lookupTransform' method in the transform listener] */
  tf_listener.lookupTransform(cfg.WORLD_FRAME_ID,cfg.TAG_FRAME_ID,ros::Time(0.0f),world_to_box_pick_tf);

  // save pose in 'box_pose'
  /* Fill Code: [ use the 'tf::poseTFToMsg' to convert a TF transform into a pose message] */
  tf::poseTFToMsg(world_to_box_pick_tf,box_pose);

  return box_pose;
}

