// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from pick_and_place_msgs:srv/GetTargetPose.idl
// generated code does not contain a copyright notice

#ifndef PICK_AND_PLACE_MSGS__SRV__DETAIL__GET_TARGET_POSE__STRUCT_HPP_
#define PICK_AND_PLACE_MSGS__SRV__DETAIL__GET_TARGET_POSE__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


// Include directives for member types
// Member 'shape'
#include "shape_msgs/msg/detail/solid_primitive__struct.hpp"
// Member 'remove_at_poses'
#include "geometry_msgs/msg/detail/pose__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__pick_and_place_msgs__srv__GetTargetPose_Request __attribute__((deprecated))
#else
# define DEPRECATED__pick_and_place_msgs__srv__GetTargetPose_Request __declspec(deprecated)
#endif

namespace pick_and_place_msgs
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct GetTargetPose_Request_
{
  using Type = GetTargetPose_Request_<ContainerAllocator>;

  explicit GetTargetPose_Request_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : shape(_init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->world_frame_id = "";
      this->ar_tag_frame_id = "";
    }
  }

  explicit GetTargetPose_Request_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : world_frame_id(_alloc),
    ar_tag_frame_id(_alloc),
    shape(_alloc, _init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->world_frame_id = "";
      this->ar_tag_frame_id = "";
    }
  }

  // field types and members
  using _world_frame_id_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _world_frame_id_type world_frame_id;
  using _ar_tag_frame_id_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _ar_tag_frame_id_type ar_tag_frame_id;
  using _shape_type =
    shape_msgs::msg::SolidPrimitive_<ContainerAllocator>;
  _shape_type shape;
  using _remove_at_poses_type =
    std::vector<geometry_msgs::msg::Pose_<ContainerAllocator>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<geometry_msgs::msg::Pose_<ContainerAllocator>>>;
  _remove_at_poses_type remove_at_poses;

  // setters for named parameter idiom
  Type & set__world_frame_id(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->world_frame_id = _arg;
    return *this;
  }
  Type & set__ar_tag_frame_id(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->ar_tag_frame_id = _arg;
    return *this;
  }
  Type & set__shape(
    const shape_msgs::msg::SolidPrimitive_<ContainerAllocator> & _arg)
  {
    this->shape = _arg;
    return *this;
  }
  Type & set__remove_at_poses(
    const std::vector<geometry_msgs::msg::Pose_<ContainerAllocator>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<geometry_msgs::msg::Pose_<ContainerAllocator>>> & _arg)
  {
    this->remove_at_poses = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    pick_and_place_msgs::srv::GetTargetPose_Request_<ContainerAllocator> *;
  using ConstRawPtr =
    const pick_and_place_msgs::srv::GetTargetPose_Request_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<pick_and_place_msgs::srv::GetTargetPose_Request_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<pick_and_place_msgs::srv::GetTargetPose_Request_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      pick_and_place_msgs::srv::GetTargetPose_Request_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<pick_and_place_msgs::srv::GetTargetPose_Request_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      pick_and_place_msgs::srv::GetTargetPose_Request_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<pick_and_place_msgs::srv::GetTargetPose_Request_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<pick_and_place_msgs::srv::GetTargetPose_Request_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<pick_and_place_msgs::srv::GetTargetPose_Request_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__pick_and_place_msgs__srv__GetTargetPose_Request
    std::shared_ptr<pick_and_place_msgs::srv::GetTargetPose_Request_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__pick_and_place_msgs__srv__GetTargetPose_Request
    std::shared_ptr<pick_and_place_msgs::srv::GetTargetPose_Request_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const GetTargetPose_Request_ & other) const
  {
    if (this->world_frame_id != other.world_frame_id) {
      return false;
    }
    if (this->ar_tag_frame_id != other.ar_tag_frame_id) {
      return false;
    }
    if (this->shape != other.shape) {
      return false;
    }
    if (this->remove_at_poses != other.remove_at_poses) {
      return false;
    }
    return true;
  }
  bool operator!=(const GetTargetPose_Request_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct GetTargetPose_Request_

// alias to use template instance with default allocator
using GetTargetPose_Request =
  pick_and_place_msgs::srv::GetTargetPose_Request_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace pick_and_place_msgs


// Include directives for member types
// Member 'target_pose'
// already included above
// #include "geometry_msgs/msg/detail/pose__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__pick_and_place_msgs__srv__GetTargetPose_Response __attribute__((deprecated))
#else
# define DEPRECATED__pick_and_place_msgs__srv__GetTargetPose_Response __declspec(deprecated)
#endif

namespace pick_and_place_msgs
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct GetTargetPose_Response_
{
  using Type = GetTargetPose_Response_<ContainerAllocator>;

  explicit GetTargetPose_Response_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : target_pose(_init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->succeeded = false;
    }
  }

  explicit GetTargetPose_Response_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : target_pose(_alloc, _init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->succeeded = false;
    }
  }

  // field types and members
  using _succeeded_type =
    bool;
  _succeeded_type succeeded;
  using _target_pose_type =
    geometry_msgs::msg::Pose_<ContainerAllocator>;
  _target_pose_type target_pose;

  // setters for named parameter idiom
  Type & set__succeeded(
    const bool & _arg)
  {
    this->succeeded = _arg;
    return *this;
  }
  Type & set__target_pose(
    const geometry_msgs::msg::Pose_<ContainerAllocator> & _arg)
  {
    this->target_pose = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    pick_and_place_msgs::srv::GetTargetPose_Response_<ContainerAllocator> *;
  using ConstRawPtr =
    const pick_and_place_msgs::srv::GetTargetPose_Response_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<pick_and_place_msgs::srv::GetTargetPose_Response_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<pick_and_place_msgs::srv::GetTargetPose_Response_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      pick_and_place_msgs::srv::GetTargetPose_Response_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<pick_and_place_msgs::srv::GetTargetPose_Response_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      pick_and_place_msgs::srv::GetTargetPose_Response_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<pick_and_place_msgs::srv::GetTargetPose_Response_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<pick_and_place_msgs::srv::GetTargetPose_Response_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<pick_and_place_msgs::srv::GetTargetPose_Response_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__pick_and_place_msgs__srv__GetTargetPose_Response
    std::shared_ptr<pick_and_place_msgs::srv::GetTargetPose_Response_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__pick_and_place_msgs__srv__GetTargetPose_Response
    std::shared_ptr<pick_and_place_msgs::srv::GetTargetPose_Response_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const GetTargetPose_Response_ & other) const
  {
    if (this->succeeded != other.succeeded) {
      return false;
    }
    if (this->target_pose != other.target_pose) {
      return false;
    }
    return true;
  }
  bool operator!=(const GetTargetPose_Response_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct GetTargetPose_Response_

// alias to use template instance with default allocator
using GetTargetPose_Response =
  pick_and_place_msgs::srv::GetTargetPose_Response_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace pick_and_place_msgs

namespace pick_and_place_msgs
{

namespace srv
{

struct GetTargetPose
{
  using Request = pick_and_place_msgs::srv::GetTargetPose_Request;
  using Response = pick_and_place_msgs::srv::GetTargetPose_Response;
};

}  // namespace srv

}  // namespace pick_and_place_msgs

#endif  // PICK_AND_PLACE_MSGS__SRV__DETAIL__GET_TARGET_POSE__STRUCT_HPP_
