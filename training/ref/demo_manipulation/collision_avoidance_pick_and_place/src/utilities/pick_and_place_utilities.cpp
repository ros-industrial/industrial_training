/*
 * pick_and_place_utilities.cpp
 *
 *  Created on: Jun 3, 2013
 *      Author: ros-industrial
 */

#include <collision_avoidance_pick_and_place/pick_and_place_utilities.h>
#include <moveit/kinematic_constraints/utils.h>
#include <tf/transform_listener.h>
#include <boost/assign/std/vector.hpp>
#include <iostream>
#include <ros/ros.h>

using namespace tf;
using namespace boost::assign;

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

moveit_msgs::Constraints create_path_orientation_constraints(const geometry_msgs::Pose &goal_pose,
		float x_tolerance,float y_tolerance,float z_tolerance,std::string link_name)
{
	moveit_msgs::Constraints path_constraints = moveit_msgs::Constraints();
	path_constraints.name = "tcp_orientation_constraint";

	// setting constraint properties
	moveit_msgs::OrientationConstraint orientation_constraint = moveit_msgs::OrientationConstraint();
	orientation_constraint.header.frame_id="world_frame";
	//orientation_constraint.orientation = goal_pose.orientation;
	orientation_constraint.orientation.w = 1;
	orientation_constraint.absolute_x_axis_tolerance = x_tolerance;
	orientation_constraint.absolute_y_axis_tolerance = y_tolerance;
	orientation_constraint.absolute_z_axis_tolerance = z_tolerance;
	orientation_constraint.weight = 1.0f;
	orientation_constraint.link_name = link_name;

	// adding orientation constraint to path_constraints object
	path_constraints.orientation_constraints.push_back(orientation_constraint);
	return path_constraints;
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
  XmlRpc::XmlRpcValue list;

  if(nh.getParam("arm_group_name",ARM_GROUP_NAME)
      && nh.getParam("tcp_link_name",TCP_LINK_NAME)
      && nh.getParam("wrist_link_name",WRIST_LINK_NAME)
      && nh.getParam("attached_object_link",ATTACHED_OBJECT_LINK_NAME)
      && nh.getParam("world_frame_id",WORLD_FRAME_ID)
      && nh.getParam("home_pose_name",HOME_POSE_NAME)
      && nh.getParam("wait_pose_name",WAIT_POSE_NAME)
      && nh.getParam("ar_frame_id",AR_TAG_FRAME_ID)
      && nh.getParam("box_width",w)
      && nh.getParam("box_length",l)
      && nh.getParam("box_height",h)
      && nh.getParam("box_place_x",x)
      && nh.getParam("box_place_y",y)
      && nh.getParam("box_place_z",z)
      && nh.getParam("touch_links",list)
      && nh.getParam("retreat_distance",RETREAT_DISTANCE)
      && nh.getParam("approach_distance",APPROACH_DISTANCE))
  {
    BOX_SIZE = Vector3(l,w,h);
    BOX_PLACE_TF.setOrigin(Vector3(x,y,z));

    // parsing touch links list
    for(int32_t i =0 ; i < list.size();i++)
    {
    	if(list[i].getType() == XmlRpc::XmlRpcValue::TypeString)
    	{
    		TOUCH_LINKS.push_back(static_cast<std::string>(list[i]));
    	}
    	else
    	{
    		ROS_ERROR_STREAM("touch links list contains invalid data.  Provide an array of strings 'touch_links:{link_a,link_b,link_c ..}'");
    	}
    }

    // building geometric primitive for target
    shape_msgs::SolidPrimitive shape;
    shape.type = shape_msgs::SolidPrimitive::BOX;
    shape.dimensions.resize(3);
    shape.dimensions[0] = BOX_SIZE.getX();
    shape.dimensions[1] = BOX_SIZE.getY();
    shape.dimensions[2] = BOX_SIZE.getZ();

    // setting pose of object relative to tcp
    tf::poseTFToMsg(tf::Transform::getIdentity(),TCP_TO_BOX_POSE);
    TCP_TO_BOX_POSE.position.x = 0;
    TCP_TO_BOX_POSE.position.y = 0;
    TCP_TO_BOX_POSE.position.z = 0.5f* BOX_SIZE.getZ();

    // creating visual object
    MARKER_MESSAGE.header.frame_id = TCP_LINK_NAME;
    MARKER_MESSAGE.type = visualization_msgs::Marker::CUBE;
    MARKER_MESSAGE.pose = TCP_TO_BOX_POSE;
    MARKER_MESSAGE.id = 0;
    MARKER_MESSAGE.color.r = 0;
    MARKER_MESSAGE.color.g = 0;
    MARKER_MESSAGE.color.b = 1;
    MARKER_MESSAGE.color.a = 0.5f;
    MARKER_MESSAGE.lifetime = ros::Duration(100); // persists forever
    MARKER_MESSAGE.frame_locked = true;
    MARKER_MESSAGE.scale.x = shape.dimensions[0];
    MARKER_MESSAGE.scale.y = shape.dimensions[1];
    MARKER_MESSAGE.scale.z = shape.dimensions[2];

    // create attached object
    ATTACHED_OBJECT.header.frame_id = TCP_LINK_NAME;
    ATTACHED_OBJECT.id=ATTACHED_OBJECT_LINK_NAME;
    ATTACHED_OBJECT.primitives.push_back(shape);
    ATTACHED_OBJECT.primitive_poses.push_back(TCP_TO_BOX_POSE);

    return true;
  }
  else
  {
    return false;
  }

}
