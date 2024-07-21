// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from pick_and_place_msgs:srv/GetTargetPose.idl
// generated code does not contain a copyright notice

#ifndef PICK_AND_PLACE_MSGS__SRV__DETAIL__GET_TARGET_POSE__BUILDER_HPP_
#define PICK_AND_PLACE_MSGS__SRV__DETAIL__GET_TARGET_POSE__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "pick_and_place_msgs/srv/detail/get_target_pose__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace pick_and_place_msgs
{

namespace srv
{

namespace builder
{

class Init_GetTargetPose_Request_remove_at_poses
{
public:
  explicit Init_GetTargetPose_Request_remove_at_poses(::pick_and_place_msgs::srv::GetTargetPose_Request & msg)
  : msg_(msg)
  {}
  ::pick_and_place_msgs::srv::GetTargetPose_Request remove_at_poses(::pick_and_place_msgs::srv::GetTargetPose_Request::_remove_at_poses_type arg)
  {
    msg_.remove_at_poses = std::move(arg);
    return std::move(msg_);
  }

private:
  ::pick_and_place_msgs::srv::GetTargetPose_Request msg_;
};

class Init_GetTargetPose_Request_shape
{
public:
  explicit Init_GetTargetPose_Request_shape(::pick_and_place_msgs::srv::GetTargetPose_Request & msg)
  : msg_(msg)
  {}
  Init_GetTargetPose_Request_remove_at_poses shape(::pick_and_place_msgs::srv::GetTargetPose_Request::_shape_type arg)
  {
    msg_.shape = std::move(arg);
    return Init_GetTargetPose_Request_remove_at_poses(msg_);
  }

private:
  ::pick_and_place_msgs::srv::GetTargetPose_Request msg_;
};

class Init_GetTargetPose_Request_ar_tag_frame_id
{
public:
  explicit Init_GetTargetPose_Request_ar_tag_frame_id(::pick_and_place_msgs::srv::GetTargetPose_Request & msg)
  : msg_(msg)
  {}
  Init_GetTargetPose_Request_shape ar_tag_frame_id(::pick_and_place_msgs::srv::GetTargetPose_Request::_ar_tag_frame_id_type arg)
  {
    msg_.ar_tag_frame_id = std::move(arg);
    return Init_GetTargetPose_Request_shape(msg_);
  }

private:
  ::pick_and_place_msgs::srv::GetTargetPose_Request msg_;
};

class Init_GetTargetPose_Request_world_frame_id
{
public:
  Init_GetTargetPose_Request_world_frame_id()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_GetTargetPose_Request_ar_tag_frame_id world_frame_id(::pick_and_place_msgs::srv::GetTargetPose_Request::_world_frame_id_type arg)
  {
    msg_.world_frame_id = std::move(arg);
    return Init_GetTargetPose_Request_ar_tag_frame_id(msg_);
  }

private:
  ::pick_and_place_msgs::srv::GetTargetPose_Request msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::pick_and_place_msgs::srv::GetTargetPose_Request>()
{
  return pick_and_place_msgs::srv::builder::Init_GetTargetPose_Request_world_frame_id();
}

}  // namespace pick_and_place_msgs


namespace pick_and_place_msgs
{

namespace srv
{

namespace builder
{

class Init_GetTargetPose_Response_target_pose
{
public:
  explicit Init_GetTargetPose_Response_target_pose(::pick_and_place_msgs::srv::GetTargetPose_Response & msg)
  : msg_(msg)
  {}
  ::pick_and_place_msgs::srv::GetTargetPose_Response target_pose(::pick_and_place_msgs::srv::GetTargetPose_Response::_target_pose_type arg)
  {
    msg_.target_pose = std::move(arg);
    return std::move(msg_);
  }

private:
  ::pick_and_place_msgs::srv::GetTargetPose_Response msg_;
};

class Init_GetTargetPose_Response_succeeded
{
public:
  Init_GetTargetPose_Response_succeeded()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_GetTargetPose_Response_target_pose succeeded(::pick_and_place_msgs::srv::GetTargetPose_Response::_succeeded_type arg)
  {
    msg_.succeeded = std::move(arg);
    return Init_GetTargetPose_Response_target_pose(msg_);
  }

private:
  ::pick_and_place_msgs::srv::GetTargetPose_Response msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::pick_and_place_msgs::srv::GetTargetPose_Response>()
{
  return pick_and_place_msgs::srv::builder::Init_GetTargetPose_Response_succeeded();
}

}  // namespace pick_and_place_msgs

#endif  // PICK_AND_PLACE_MSGS__SRV__DETAIL__GET_TARGET_POSE__BUILDER_HPP_
