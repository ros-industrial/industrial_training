/*
 * pick_and_place_utilities.cpp
 *
 *  Created on: Jun 3, 2013
 *      Author: ros-industrial
 */

#include <iostream>

#include <rclcpp/rclcpp.hpp>

#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_listener.h>

#include <moveit/kinematic_constraints/utils.h>

#include <pick_and_place_application/pick_and_place_utilities.h>

using namespace tf2;

// =============================== Utility functions ===============================
namespace tf2
{
geometry_msgs::msg::Pose transformToPoseMsg(const tf2::Transform& t_in)
{
  geometry_msgs::msg::Pose pose;
  pose.position.x = t_in.getOrigin().x();
  pose.position.y = t_in.getOrigin().y();
  pose.position.z = t_in.getOrigin().z();
  pose.orientation = tf2::toMsg(t_in.getRotation());
  return pose;
}

tf2::Transform poseMsgToTransform(const geometry_msgs::msg::Pose& pose_in)
{
  tf2::Transform t_out;
  tf2::Vector3 pos(pose_in.position.x, pose_in.position.y, pose_in.position.z);
  t_out.setOrigin(pos);
  tf2::Quaternion q;
  tf2::fromMsg(pose_in.orientation, q);
  t_out.setRotation(q);
  return t_out;
}
}  // namespace tf2

std::vector<geometry_msgs::msg::Pose> createManipulationPoses(double retreat_dis,
                                                              double approach_dis,
                                                              const tf2::Transform& target_tf)
{
  geometry_msgs::msg::Pose start_pose, target_pose, end_pose;
  std::vector<geometry_msgs::msg::Pose> poses;

  // creating start pose by applying a translation along +z by approach distance
  tf2::toMsg(Transform(Quaternion::getIdentity(), Vector3(0, 0, approach_dis)) * target_tf, start_pose);

  // converting target pose
  tf2::toMsg(target_tf, target_pose);

  // creating end pose by applying a translation along +z by retreat distance
  tf2::toMsg(Transform(Quaternion::getIdentity(), Vector3(0, 0, retreat_dis)) * target_tf, end_pose);

  poses.clear();
  poses.push_back(start_pose);
  poses.push_back(target_pose);
  poses.push_back(end_pose);

  return poses;
}

std::vector<geometry_msgs::msg::Pose> applyTransform(tf2::Transform tcp_to_wrist_tf,
                                                     const std::vector<geometry_msgs::msg::Pose> tcp_poses)
{
  // array for poses of the wrist
  std::vector<geometry_msgs::msg::Pose> wrist_poses;
  wrist_poses.resize(tcp_poses.size());

  // applying transform to each tcp poses
  tf2::Transform world_to_wrist_tf, world_to_tcp_tf;
  wrist_poses.resize(tcp_poses.size());
  for (unsigned int i = 0; i < tcp_poses.size(); i++)
  {
    tf2::fromMsg(tcp_poses[i], world_to_tcp_tf);
    world_to_wrist_tf = world_to_tcp_tf * tcp_to_wrist_tf;
    // tf2::fromMsg(world_to_wrist_tf,wrist_poses[i]);
    wrist_poses[i] = tf2::transformToPoseMsg(world_to_wrist_tf);
  }

  return wrist_poses;
}

moveit_msgs::msg::Constraints createPathOrientationConstraints(const geometry_msgs::msg::Pose& goal_pose,
                                                               float x_tolerance,
                                                               float y_tolerance,
                                                               float z_tolerance,
                                                               std::string link_name)
{
  moveit_msgs::msg::Constraints path_constraints = moveit_msgs::msg::Constraints();
  path_constraints.name = "tcp_orientation_constraint";

  // setting constraint properties
  moveit_msgs::msg::OrientationConstraint orientation_constraint = moveit_msgs::msg::OrientationConstraint();
  orientation_constraint.header.frame_id = "world_frame";
  orientation_constraint.orientation = goal_pose.orientation;
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

std::ostream& operator<<(std::ostream& os, const tf2::Vector3 vec)
{
  return os << "[" << vec.getX() << ", " << vec.getY() << ", " << vec.getZ() << "]";
}

std::ostream& operator<<(std::ostream& os, const geometry_msgs::msg::Point pt)
{
  return os << "[" << pt.x << ", " << pt.y << ", " << pt.z << "]";
}

template <class T>
bool loadParameter(rclcpp::Node::SharedPtr node, const std::string& param_name, T& param_val)
{
  if (!node->has_parameter(param_name))
  {
    RCLCPP_INFO(node->get_logger(), "The \"%s\" parameter was not found", param_name.c_str());
    return false;
  }
  return node->get_parameter(param_name, param_val);
}

bool PickAndPlaceConfig::init(rclcpp::Node::SharedPtr node)
{
  double w, l, h, x, y, z;
  if (loadParameter(node, "arm_group_name", ARM_GROUP_NAME) && loadParameter(node, "tcp_link_name", TCP_LINK_NAME) &&
      loadParameter(node, "attached_object_link", ATTACHED_OBJECT_LINK_NAME) &&
      loadParameter(node, "world_frame_id", WORLD_FRAME_ID) && loadParameter(node, "home_pose_name", HOME_POSE_NAME) &&
      loadParameter(node, "wait_pose_name", WAIT_POSE_NAME) && loadParameter(node, "ar_frame_id", AR_TAG_FRAME_ID) &&
      loadParameter(node, "box_width", w) && loadParameter(node, "box_length", l) &&
      loadParameter(node, "box_height", h) && loadParameter(node, "box_place_x", x) &&
      loadParameter(node, "box_place_y", y) && loadParameter(node, "box_place_z", z) &&
      loadParameter(node, "touch_links", TOUCH_LINKS) && loadParameter(node, "retreat_distance", RETREAT_DISTANCE) &&
      loadParameter(node, "approach_distance", APPROACH_DISTANCE) && loadParameter(node, "planner_id", PLANNER_ID) &&
      loadParameter(node, "planning_attempts", PLANNING_ATTEMPTS) &&
      loadParameter(node, "planning_time", PLANNING_TIME))
  {
    BOX_SIZE = Vector3(l, w, h);
    BOX_PLACE_TF.setOrigin(Vector3(x, y, z));

    // building geometric primitive for target
    shape_msgs::msg::SolidPrimitive shape;
    shape.type = shape_msgs::msg::SolidPrimitive::BOX;
    shape.dimensions.resize(3);
    shape.dimensions[0] = BOX_SIZE.getX();
    shape.dimensions[1] = BOX_SIZE.getY();
    shape.dimensions[2] = BOX_SIZE.getZ();

    // setting pose of object relative to tcp
    TCP_TO_BOX_POSE = tf2::transformToPoseMsg(tf2::Transform::getIdentity());
    TCP_TO_BOX_POSE.position.x = 0;
    TCP_TO_BOX_POSE.position.y = 0;
    TCP_TO_BOX_POSE.position.z = 0.5f * BOX_SIZE.getZ();

    // creating visual object
    MARKER_MESSAGE.header.frame_id = TCP_LINK_NAME;
    MARKER_MESSAGE.type = visualization_msgs::msg::Marker::CUBE;
    MARKER_MESSAGE.pose = TCP_TO_BOX_POSE;
    MARKER_MESSAGE.id = 0;
    MARKER_MESSAGE.color.r = 0;
    MARKER_MESSAGE.color.g = 0;
    MARKER_MESSAGE.color.b = 1;
    MARKER_MESSAGE.color.a = 0.5f;
    MARKER_MESSAGE.lifetime = rclcpp::Duration::from_seconds(0);  // persists forever
    MARKER_MESSAGE.frame_locked = true;
    MARKER_MESSAGE.scale.x = shape.dimensions[0];
    MARKER_MESSAGE.scale.y = shape.dimensions[1];
    MARKER_MESSAGE.scale.z = shape.dimensions[2];

    // create attached object
    ATTACHED_OBJECT.header.frame_id = TCP_LINK_NAME;
    ATTACHED_OBJECT.id = ATTACHED_OBJECT_LINK_NAME;
    ATTACHED_OBJECT.primitives.push_back(shape);
    ATTACHED_OBJECT.primitive_poses.push_back(TCP_TO_BOX_POSE);

    return true;
  }
  else
  {
    return false;
  }
}
