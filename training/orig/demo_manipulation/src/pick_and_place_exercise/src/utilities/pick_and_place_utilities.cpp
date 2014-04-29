/*
 * pick_and_place_utilities.cpp
 *
 *  Created on: Jun 3, 2013
 *      Author: ros-industrial
 */

#include <pick_and_place_exercise/pick_and_place_utilities.h>
#include <iostream>
#include <ros/ros.h>

using namespace tf;

// =============================== Utility functions ===============================

std::vector<geometry_msgs::Pose> create_manipulation_poses(double retreat_dis,double approach_dis,const tf::Transform &target_tf)
{
  geometry_msgs::Pose start_pose, target_pose, end_pose;
  std::vector<geometry_msgs::Pose> poses;

  // creating start pose by applying a translation along +z by approach distance
  tf::poseTFToMsg(Transform(Quaternion::getIdentity(),Vector3(0,0,approach_dis))*target_tf,start_pose);

  // converting target pose
  tf::poseTFToMsg(target_tf,target_pose);

  // creating end pose by applying a translation along +z by retreat distance
  tf::poseTFToMsg(Transform(Quaternion::getIdentity(),Vector3(0,0,retreat_dis))*target_tf,end_pose);

  poses.clear();
  poses.push_back(start_pose);
  poses.push_back(target_pose);
  poses.push_back(end_pose);

  return poses;
}

std::vector<geometry_msgs::Pose> transform_from_tcp_to_wrist(tf::Transform tcp_to_wrist_tf,const std::vector<geometry_msgs::Pose> tcp_poses)
{
  // array for poses of the wrist
  std::vector<geometry_msgs::Pose> wrist_poses;
  wrist_poses.resize(tcp_poses.size());

  // applying transform to each tcp poses
  tf::Transform world_to_wrist_tf, world_to_tcp_tf;
  wrist_poses.resize(tcp_poses.size());
  for(unsigned int i = 0; i < tcp_poses.size(); i++)
  {
    tf::poseMsgToTF(tcp_poses[i],world_to_tcp_tf);
    world_to_wrist_tf = world_to_tcp_tf * tcp_to_wrist_tf;
    tf::poseTFToMsg(world_to_wrist_tf,wrist_poses[i]);
  }

  return wrist_poses;
}

std::ostream& operator<<(std::ostream& os, const tf::Vector3 vec)
{
  return os << "[" << vec.getX() << ", " << vec.getY() << ", " << vec.getZ() << "]";
}

std::ostream& operator<<(std::ostream& os, const geometry_msgs::Point pt)
{
  return os << "[" << pt.x << ", " << pt.y << ", " << pt.z << "]";
}

bool pick_and_place_config::init()
{
  ros::NodeHandle nh("~");
  double w, l, h, x, y, z;

  if(nh.getParam("arm_group_name",ARM_GROUP_NAME)
      && nh.getParam("tcp_link_name",TCP_LINK_NAME)
      && nh.getParam("wrist_link_name",WRIST_LINK_NAME)
      && nh.getParam("world_frame_id",WORLD_FRAME_ID)
      && nh.getParam("home_pose_name",HOME_POSE_NAME)
      && nh.getParam("wait_pose_name",WAIT_POSE_NAME)
      && nh.getParam("tag_frame_id",TAG_FRAME_ID)
      && nh.getParam("box_width",w)
      && nh.getParam("box_length",l)
      && nh.getParam("box_height",h)
      && nh.getParam("box_place_x",x)
      && nh.getParam("box_place_y",y)
      && nh.getParam("box_place_z",z)
      && nh.getParam("retreat_distance",RETREAT_DISTANCE)
      && nh.getParam("approach_distance",APPROACH_DISTANCE))
  {
    BOX_SIZE = Vector3(l,w,h);
    BOX_PLACE_TF.setOrigin(Vector3(x,y,z));
    return true;
  }
  else
  {
    return false;
  }

}
