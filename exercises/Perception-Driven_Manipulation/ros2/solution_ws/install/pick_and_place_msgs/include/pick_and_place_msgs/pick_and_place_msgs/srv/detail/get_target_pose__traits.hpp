// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from pick_and_place_msgs:srv/GetTargetPose.idl
// generated code does not contain a copyright notice

#ifndef PICK_AND_PLACE_MSGS__SRV__DETAIL__GET_TARGET_POSE__TRAITS_HPP_
#define PICK_AND_PLACE_MSGS__SRV__DETAIL__GET_TARGET_POSE__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "pick_and_place_msgs/srv/detail/get_target_pose__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

// Include directives for member types
// Member 'shape'
#include "shape_msgs/msg/detail/solid_primitive__traits.hpp"
// Member 'remove_at_poses'
#include "geometry_msgs/msg/detail/pose__traits.hpp"

namespace pick_and_place_msgs
{

namespace srv
{

inline void to_flow_style_yaml(
  const GetTargetPose_Request & msg,
  std::ostream & out)
{
  out << "{";
  // member: world_frame_id
  {
    out << "world_frame_id: ";
    rosidl_generator_traits::value_to_yaml(msg.world_frame_id, out);
    out << ", ";
  }

  // member: ar_tag_frame_id
  {
    out << "ar_tag_frame_id: ";
    rosidl_generator_traits::value_to_yaml(msg.ar_tag_frame_id, out);
    out << ", ";
  }

  // member: shape
  {
    out << "shape: ";
    to_flow_style_yaml(msg.shape, out);
    out << ", ";
  }

  // member: remove_at_poses
  {
    if (msg.remove_at_poses.size() == 0) {
      out << "remove_at_poses: []";
    } else {
      out << "remove_at_poses: [";
      size_t pending_items = msg.remove_at_poses.size();
      for (auto item : msg.remove_at_poses) {
        to_flow_style_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const GetTargetPose_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: world_frame_id
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "world_frame_id: ";
    rosidl_generator_traits::value_to_yaml(msg.world_frame_id, out);
    out << "\n";
  }

  // member: ar_tag_frame_id
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "ar_tag_frame_id: ";
    rosidl_generator_traits::value_to_yaml(msg.ar_tag_frame_id, out);
    out << "\n";
  }

  // member: shape
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "shape:\n";
    to_block_style_yaml(msg.shape, out, indentation + 2);
  }

  // member: remove_at_poses
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.remove_at_poses.size() == 0) {
      out << "remove_at_poses: []\n";
    } else {
      out << "remove_at_poses:\n";
      for (auto item : msg.remove_at_poses) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "-\n";
        to_block_style_yaml(item, out, indentation + 2);
      }
    }
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const GetTargetPose_Request & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace srv

}  // namespace pick_and_place_msgs

namespace rosidl_generator_traits
{

[[deprecated("use pick_and_place_msgs::srv::to_block_style_yaml() instead")]]
inline void to_yaml(
  const pick_and_place_msgs::srv::GetTargetPose_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  pick_and_place_msgs::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use pick_and_place_msgs::srv::to_yaml() instead")]]
inline std::string to_yaml(const pick_and_place_msgs::srv::GetTargetPose_Request & msg)
{
  return pick_and_place_msgs::srv::to_yaml(msg);
}

template<>
inline const char * data_type<pick_and_place_msgs::srv::GetTargetPose_Request>()
{
  return "pick_and_place_msgs::srv::GetTargetPose_Request";
}

template<>
inline const char * name<pick_and_place_msgs::srv::GetTargetPose_Request>()
{
  return "pick_and_place_msgs/srv/GetTargetPose_Request";
}

template<>
struct has_fixed_size<pick_and_place_msgs::srv::GetTargetPose_Request>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<pick_and_place_msgs::srv::GetTargetPose_Request>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<pick_and_place_msgs::srv::GetTargetPose_Request>
  : std::true_type {};

}  // namespace rosidl_generator_traits

// Include directives for member types
// Member 'target_pose'
// already included above
// #include "geometry_msgs/msg/detail/pose__traits.hpp"

namespace pick_and_place_msgs
{

namespace srv
{

inline void to_flow_style_yaml(
  const GetTargetPose_Response & msg,
  std::ostream & out)
{
  out << "{";
  // member: succeeded
  {
    out << "succeeded: ";
    rosidl_generator_traits::value_to_yaml(msg.succeeded, out);
    out << ", ";
  }

  // member: target_pose
  {
    out << "target_pose: ";
    to_flow_style_yaml(msg.target_pose, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const GetTargetPose_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: succeeded
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "succeeded: ";
    rosidl_generator_traits::value_to_yaml(msg.succeeded, out);
    out << "\n";
  }

  // member: target_pose
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "target_pose:\n";
    to_block_style_yaml(msg.target_pose, out, indentation + 2);
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const GetTargetPose_Response & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace srv

}  // namespace pick_and_place_msgs

namespace rosidl_generator_traits
{

[[deprecated("use pick_and_place_msgs::srv::to_block_style_yaml() instead")]]
inline void to_yaml(
  const pick_and_place_msgs::srv::GetTargetPose_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  pick_and_place_msgs::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use pick_and_place_msgs::srv::to_yaml() instead")]]
inline std::string to_yaml(const pick_and_place_msgs::srv::GetTargetPose_Response & msg)
{
  return pick_and_place_msgs::srv::to_yaml(msg);
}

template<>
inline const char * data_type<pick_and_place_msgs::srv::GetTargetPose_Response>()
{
  return "pick_and_place_msgs::srv::GetTargetPose_Response";
}

template<>
inline const char * name<pick_and_place_msgs::srv::GetTargetPose_Response>()
{
  return "pick_and_place_msgs/srv/GetTargetPose_Response";
}

template<>
struct has_fixed_size<pick_and_place_msgs::srv::GetTargetPose_Response>
  : std::integral_constant<bool, has_fixed_size<geometry_msgs::msg::Pose>::value> {};

template<>
struct has_bounded_size<pick_and_place_msgs::srv::GetTargetPose_Response>
  : std::integral_constant<bool, has_bounded_size<geometry_msgs::msg::Pose>::value> {};

template<>
struct is_message<pick_and_place_msgs::srv::GetTargetPose_Response>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<pick_and_place_msgs::srv::GetTargetPose>()
{
  return "pick_and_place_msgs::srv::GetTargetPose";
}

template<>
inline const char * name<pick_and_place_msgs::srv::GetTargetPose>()
{
  return "pick_and_place_msgs/srv/GetTargetPose";
}

template<>
struct has_fixed_size<pick_and_place_msgs::srv::GetTargetPose>
  : std::integral_constant<
    bool,
    has_fixed_size<pick_and_place_msgs::srv::GetTargetPose_Request>::value &&
    has_fixed_size<pick_and_place_msgs::srv::GetTargetPose_Response>::value
  >
{
};

template<>
struct has_bounded_size<pick_and_place_msgs::srv::GetTargetPose>
  : std::integral_constant<
    bool,
    has_bounded_size<pick_and_place_msgs::srv::GetTargetPose_Request>::value &&
    has_bounded_size<pick_and_place_msgs::srv::GetTargetPose_Response>::value
  >
{
};

template<>
struct is_service<pick_and_place_msgs::srv::GetTargetPose>
  : std::true_type
{
};

template<>
struct is_service_request<pick_and_place_msgs::srv::GetTargetPose_Request>
  : std::true_type
{
};

template<>
struct is_service_response<pick_and_place_msgs::srv::GetTargetPose_Response>
  : std::true_type
{
};

}  // namespace rosidl_generator_traits

#endif  // PICK_AND_PLACE_MSGS__SRV__DETAIL__GET_TARGET_POSE__TRAITS_HPP_
