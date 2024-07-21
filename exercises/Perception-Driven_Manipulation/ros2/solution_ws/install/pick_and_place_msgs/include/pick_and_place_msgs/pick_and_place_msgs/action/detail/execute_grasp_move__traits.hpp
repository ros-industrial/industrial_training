// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from pick_and_place_msgs:action/ExecuteGraspMove.idl
// generated code does not contain a copyright notice

#ifndef PICK_AND_PLACE_MSGS__ACTION__DETAIL__EXECUTE_GRASP_MOVE__TRAITS_HPP_
#define PICK_AND_PLACE_MSGS__ACTION__DETAIL__EXECUTE_GRASP_MOVE__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "pick_and_place_msgs/action/detail/execute_grasp_move__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace pick_and_place_msgs
{

namespace action
{

inline void to_flow_style_yaml(
  const ExecuteGraspMove_Goal & msg,
  std::ostream & out)
{
  out << "{";
  // member: goal
  {
    out << "goal: ";
    rosidl_generator_traits::value_to_yaml(msg.goal, out);
    out << ", ";
  }

  // member: max_contact_force
  {
    out << "max_contact_force: ";
    rosidl_generator_traits::value_to_yaml(msg.max_contact_force, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const ExecuteGraspMove_Goal & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: goal
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "goal: ";
    rosidl_generator_traits::value_to_yaml(msg.goal, out);
    out << "\n";
  }

  // member: max_contact_force
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "max_contact_force: ";
    rosidl_generator_traits::value_to_yaml(msg.max_contact_force, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const ExecuteGraspMove_Goal & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace action

}  // namespace pick_and_place_msgs

namespace rosidl_generator_traits
{

[[deprecated("use pick_and_place_msgs::action::to_block_style_yaml() instead")]]
inline void to_yaml(
  const pick_and_place_msgs::action::ExecuteGraspMove_Goal & msg,
  std::ostream & out, size_t indentation = 0)
{
  pick_and_place_msgs::action::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use pick_and_place_msgs::action::to_yaml() instead")]]
inline std::string to_yaml(const pick_and_place_msgs::action::ExecuteGraspMove_Goal & msg)
{
  return pick_and_place_msgs::action::to_yaml(msg);
}

template<>
inline const char * data_type<pick_and_place_msgs::action::ExecuteGraspMove_Goal>()
{
  return "pick_and_place_msgs::action::ExecuteGraspMove_Goal";
}

template<>
inline const char * name<pick_and_place_msgs::action::ExecuteGraspMove_Goal>()
{
  return "pick_and_place_msgs/action/ExecuteGraspMove_Goal";
}

template<>
struct has_fixed_size<pick_and_place_msgs::action::ExecuteGraspMove_Goal>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<pick_and_place_msgs::action::ExecuteGraspMove_Goal>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<pick_and_place_msgs::action::ExecuteGraspMove_Goal>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace pick_and_place_msgs
{

namespace action
{

inline void to_flow_style_yaml(
  const ExecuteGraspMove_Result & msg,
  std::ostream & out)
{
  out << "{";
  // member: success
  {
    out << "success: ";
    rosidl_generator_traits::value_to_yaml(msg.success, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const ExecuteGraspMove_Result & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: success
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "success: ";
    rosidl_generator_traits::value_to_yaml(msg.success, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const ExecuteGraspMove_Result & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace action

}  // namespace pick_and_place_msgs

namespace rosidl_generator_traits
{

[[deprecated("use pick_and_place_msgs::action::to_block_style_yaml() instead")]]
inline void to_yaml(
  const pick_and_place_msgs::action::ExecuteGraspMove_Result & msg,
  std::ostream & out, size_t indentation = 0)
{
  pick_and_place_msgs::action::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use pick_and_place_msgs::action::to_yaml() instead")]]
inline std::string to_yaml(const pick_and_place_msgs::action::ExecuteGraspMove_Result & msg)
{
  return pick_and_place_msgs::action::to_yaml(msg);
}

template<>
inline const char * data_type<pick_and_place_msgs::action::ExecuteGraspMove_Result>()
{
  return "pick_and_place_msgs::action::ExecuteGraspMove_Result";
}

template<>
inline const char * name<pick_and_place_msgs::action::ExecuteGraspMove_Result>()
{
  return "pick_and_place_msgs/action/ExecuteGraspMove_Result";
}

template<>
struct has_fixed_size<pick_and_place_msgs::action::ExecuteGraspMove_Result>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<pick_and_place_msgs::action::ExecuteGraspMove_Result>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<pick_and_place_msgs::action::ExecuteGraspMove_Result>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace pick_and_place_msgs
{

namespace action
{

inline void to_flow_style_yaml(
  const ExecuteGraspMove_Feedback & msg,
  std::ostream & out)
{
  (void)msg;
  out << "null";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const ExecuteGraspMove_Feedback & msg,
  std::ostream & out, size_t indentation = 0)
{
  (void)msg;
  (void)indentation;
  out << "null\n";
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const ExecuteGraspMove_Feedback & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace action

}  // namespace pick_and_place_msgs

namespace rosidl_generator_traits
{

[[deprecated("use pick_and_place_msgs::action::to_block_style_yaml() instead")]]
inline void to_yaml(
  const pick_and_place_msgs::action::ExecuteGraspMove_Feedback & msg,
  std::ostream & out, size_t indentation = 0)
{
  pick_and_place_msgs::action::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use pick_and_place_msgs::action::to_yaml() instead")]]
inline std::string to_yaml(const pick_and_place_msgs::action::ExecuteGraspMove_Feedback & msg)
{
  return pick_and_place_msgs::action::to_yaml(msg);
}

template<>
inline const char * data_type<pick_and_place_msgs::action::ExecuteGraspMove_Feedback>()
{
  return "pick_and_place_msgs::action::ExecuteGraspMove_Feedback";
}

template<>
inline const char * name<pick_and_place_msgs::action::ExecuteGraspMove_Feedback>()
{
  return "pick_and_place_msgs/action/ExecuteGraspMove_Feedback";
}

template<>
struct has_fixed_size<pick_and_place_msgs::action::ExecuteGraspMove_Feedback>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<pick_and_place_msgs::action::ExecuteGraspMove_Feedback>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<pick_and_place_msgs::action::ExecuteGraspMove_Feedback>
  : std::true_type {};

}  // namespace rosidl_generator_traits

// Include directives for member types
// Member 'goal_id'
#include "unique_identifier_msgs/msg/detail/uuid__traits.hpp"
// Member 'goal'
#include "pick_and_place_msgs/action/detail/execute_grasp_move__traits.hpp"

namespace pick_and_place_msgs
{

namespace action
{

inline void to_flow_style_yaml(
  const ExecuteGraspMove_SendGoal_Request & msg,
  std::ostream & out)
{
  out << "{";
  // member: goal_id
  {
    out << "goal_id: ";
    to_flow_style_yaml(msg.goal_id, out);
    out << ", ";
  }

  // member: goal
  {
    out << "goal: ";
    to_flow_style_yaml(msg.goal, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const ExecuteGraspMove_SendGoal_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: goal_id
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "goal_id:\n";
    to_block_style_yaml(msg.goal_id, out, indentation + 2);
  }

  // member: goal
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "goal:\n";
    to_block_style_yaml(msg.goal, out, indentation + 2);
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const ExecuteGraspMove_SendGoal_Request & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace action

}  // namespace pick_and_place_msgs

namespace rosidl_generator_traits
{

[[deprecated("use pick_and_place_msgs::action::to_block_style_yaml() instead")]]
inline void to_yaml(
  const pick_and_place_msgs::action::ExecuteGraspMove_SendGoal_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  pick_and_place_msgs::action::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use pick_and_place_msgs::action::to_yaml() instead")]]
inline std::string to_yaml(const pick_and_place_msgs::action::ExecuteGraspMove_SendGoal_Request & msg)
{
  return pick_and_place_msgs::action::to_yaml(msg);
}

template<>
inline const char * data_type<pick_and_place_msgs::action::ExecuteGraspMove_SendGoal_Request>()
{
  return "pick_and_place_msgs::action::ExecuteGraspMove_SendGoal_Request";
}

template<>
inline const char * name<pick_and_place_msgs::action::ExecuteGraspMove_SendGoal_Request>()
{
  return "pick_and_place_msgs/action/ExecuteGraspMove_SendGoal_Request";
}

template<>
struct has_fixed_size<pick_and_place_msgs::action::ExecuteGraspMove_SendGoal_Request>
  : std::integral_constant<bool, has_fixed_size<pick_and_place_msgs::action::ExecuteGraspMove_Goal>::value && has_fixed_size<unique_identifier_msgs::msg::UUID>::value> {};

template<>
struct has_bounded_size<pick_and_place_msgs::action::ExecuteGraspMove_SendGoal_Request>
  : std::integral_constant<bool, has_bounded_size<pick_and_place_msgs::action::ExecuteGraspMove_Goal>::value && has_bounded_size<unique_identifier_msgs::msg::UUID>::value> {};

template<>
struct is_message<pick_and_place_msgs::action::ExecuteGraspMove_SendGoal_Request>
  : std::true_type {};

}  // namespace rosidl_generator_traits

// Include directives for member types
// Member 'stamp'
#include "builtin_interfaces/msg/detail/time__traits.hpp"

namespace pick_and_place_msgs
{

namespace action
{

inline void to_flow_style_yaml(
  const ExecuteGraspMove_SendGoal_Response & msg,
  std::ostream & out)
{
  out << "{";
  // member: accepted
  {
    out << "accepted: ";
    rosidl_generator_traits::value_to_yaml(msg.accepted, out);
    out << ", ";
  }

  // member: stamp
  {
    out << "stamp: ";
    to_flow_style_yaml(msg.stamp, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const ExecuteGraspMove_SendGoal_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: accepted
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "accepted: ";
    rosidl_generator_traits::value_to_yaml(msg.accepted, out);
    out << "\n";
  }

  // member: stamp
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "stamp:\n";
    to_block_style_yaml(msg.stamp, out, indentation + 2);
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const ExecuteGraspMove_SendGoal_Response & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace action

}  // namespace pick_and_place_msgs

namespace rosidl_generator_traits
{

[[deprecated("use pick_and_place_msgs::action::to_block_style_yaml() instead")]]
inline void to_yaml(
  const pick_and_place_msgs::action::ExecuteGraspMove_SendGoal_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  pick_and_place_msgs::action::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use pick_and_place_msgs::action::to_yaml() instead")]]
inline std::string to_yaml(const pick_and_place_msgs::action::ExecuteGraspMove_SendGoal_Response & msg)
{
  return pick_and_place_msgs::action::to_yaml(msg);
}

template<>
inline const char * data_type<pick_and_place_msgs::action::ExecuteGraspMove_SendGoal_Response>()
{
  return "pick_and_place_msgs::action::ExecuteGraspMove_SendGoal_Response";
}

template<>
inline const char * name<pick_and_place_msgs::action::ExecuteGraspMove_SendGoal_Response>()
{
  return "pick_and_place_msgs/action/ExecuteGraspMove_SendGoal_Response";
}

template<>
struct has_fixed_size<pick_and_place_msgs::action::ExecuteGraspMove_SendGoal_Response>
  : std::integral_constant<bool, has_fixed_size<builtin_interfaces::msg::Time>::value> {};

template<>
struct has_bounded_size<pick_and_place_msgs::action::ExecuteGraspMove_SendGoal_Response>
  : std::integral_constant<bool, has_bounded_size<builtin_interfaces::msg::Time>::value> {};

template<>
struct is_message<pick_and_place_msgs::action::ExecuteGraspMove_SendGoal_Response>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<pick_and_place_msgs::action::ExecuteGraspMove_SendGoal>()
{
  return "pick_and_place_msgs::action::ExecuteGraspMove_SendGoal";
}

template<>
inline const char * name<pick_and_place_msgs::action::ExecuteGraspMove_SendGoal>()
{
  return "pick_and_place_msgs/action/ExecuteGraspMove_SendGoal";
}

template<>
struct has_fixed_size<pick_and_place_msgs::action::ExecuteGraspMove_SendGoal>
  : std::integral_constant<
    bool,
    has_fixed_size<pick_and_place_msgs::action::ExecuteGraspMove_SendGoal_Request>::value &&
    has_fixed_size<pick_and_place_msgs::action::ExecuteGraspMove_SendGoal_Response>::value
  >
{
};

template<>
struct has_bounded_size<pick_and_place_msgs::action::ExecuteGraspMove_SendGoal>
  : std::integral_constant<
    bool,
    has_bounded_size<pick_and_place_msgs::action::ExecuteGraspMove_SendGoal_Request>::value &&
    has_bounded_size<pick_and_place_msgs::action::ExecuteGraspMove_SendGoal_Response>::value
  >
{
};

template<>
struct is_service<pick_and_place_msgs::action::ExecuteGraspMove_SendGoal>
  : std::true_type
{
};

template<>
struct is_service_request<pick_and_place_msgs::action::ExecuteGraspMove_SendGoal_Request>
  : std::true_type
{
};

template<>
struct is_service_response<pick_and_place_msgs::action::ExecuteGraspMove_SendGoal_Response>
  : std::true_type
{
};

}  // namespace rosidl_generator_traits

// Include directives for member types
// Member 'goal_id'
// already included above
// #include "unique_identifier_msgs/msg/detail/uuid__traits.hpp"

namespace pick_and_place_msgs
{

namespace action
{

inline void to_flow_style_yaml(
  const ExecuteGraspMove_GetResult_Request & msg,
  std::ostream & out)
{
  out << "{";
  // member: goal_id
  {
    out << "goal_id: ";
    to_flow_style_yaml(msg.goal_id, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const ExecuteGraspMove_GetResult_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: goal_id
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "goal_id:\n";
    to_block_style_yaml(msg.goal_id, out, indentation + 2);
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const ExecuteGraspMove_GetResult_Request & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace action

}  // namespace pick_and_place_msgs

namespace rosidl_generator_traits
{

[[deprecated("use pick_and_place_msgs::action::to_block_style_yaml() instead")]]
inline void to_yaml(
  const pick_and_place_msgs::action::ExecuteGraspMove_GetResult_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  pick_and_place_msgs::action::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use pick_and_place_msgs::action::to_yaml() instead")]]
inline std::string to_yaml(const pick_and_place_msgs::action::ExecuteGraspMove_GetResult_Request & msg)
{
  return pick_and_place_msgs::action::to_yaml(msg);
}

template<>
inline const char * data_type<pick_and_place_msgs::action::ExecuteGraspMove_GetResult_Request>()
{
  return "pick_and_place_msgs::action::ExecuteGraspMove_GetResult_Request";
}

template<>
inline const char * name<pick_and_place_msgs::action::ExecuteGraspMove_GetResult_Request>()
{
  return "pick_and_place_msgs/action/ExecuteGraspMove_GetResult_Request";
}

template<>
struct has_fixed_size<pick_and_place_msgs::action::ExecuteGraspMove_GetResult_Request>
  : std::integral_constant<bool, has_fixed_size<unique_identifier_msgs::msg::UUID>::value> {};

template<>
struct has_bounded_size<pick_and_place_msgs::action::ExecuteGraspMove_GetResult_Request>
  : std::integral_constant<bool, has_bounded_size<unique_identifier_msgs::msg::UUID>::value> {};

template<>
struct is_message<pick_and_place_msgs::action::ExecuteGraspMove_GetResult_Request>
  : std::true_type {};

}  // namespace rosidl_generator_traits

// Include directives for member types
// Member 'result'
// already included above
// #include "pick_and_place_msgs/action/detail/execute_grasp_move__traits.hpp"

namespace pick_and_place_msgs
{

namespace action
{

inline void to_flow_style_yaml(
  const ExecuteGraspMove_GetResult_Response & msg,
  std::ostream & out)
{
  out << "{";
  // member: status
  {
    out << "status: ";
    rosidl_generator_traits::value_to_yaml(msg.status, out);
    out << ", ";
  }

  // member: result
  {
    out << "result: ";
    to_flow_style_yaml(msg.result, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const ExecuteGraspMove_GetResult_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: status
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "status: ";
    rosidl_generator_traits::value_to_yaml(msg.status, out);
    out << "\n";
  }

  // member: result
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "result:\n";
    to_block_style_yaml(msg.result, out, indentation + 2);
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const ExecuteGraspMove_GetResult_Response & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace action

}  // namespace pick_and_place_msgs

namespace rosidl_generator_traits
{

[[deprecated("use pick_and_place_msgs::action::to_block_style_yaml() instead")]]
inline void to_yaml(
  const pick_and_place_msgs::action::ExecuteGraspMove_GetResult_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  pick_and_place_msgs::action::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use pick_and_place_msgs::action::to_yaml() instead")]]
inline std::string to_yaml(const pick_and_place_msgs::action::ExecuteGraspMove_GetResult_Response & msg)
{
  return pick_and_place_msgs::action::to_yaml(msg);
}

template<>
inline const char * data_type<pick_and_place_msgs::action::ExecuteGraspMove_GetResult_Response>()
{
  return "pick_and_place_msgs::action::ExecuteGraspMove_GetResult_Response";
}

template<>
inline const char * name<pick_and_place_msgs::action::ExecuteGraspMove_GetResult_Response>()
{
  return "pick_and_place_msgs/action/ExecuteGraspMove_GetResult_Response";
}

template<>
struct has_fixed_size<pick_and_place_msgs::action::ExecuteGraspMove_GetResult_Response>
  : std::integral_constant<bool, has_fixed_size<pick_and_place_msgs::action::ExecuteGraspMove_Result>::value> {};

template<>
struct has_bounded_size<pick_and_place_msgs::action::ExecuteGraspMove_GetResult_Response>
  : std::integral_constant<bool, has_bounded_size<pick_and_place_msgs::action::ExecuteGraspMove_Result>::value> {};

template<>
struct is_message<pick_and_place_msgs::action::ExecuteGraspMove_GetResult_Response>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<pick_and_place_msgs::action::ExecuteGraspMove_GetResult>()
{
  return "pick_and_place_msgs::action::ExecuteGraspMove_GetResult";
}

template<>
inline const char * name<pick_and_place_msgs::action::ExecuteGraspMove_GetResult>()
{
  return "pick_and_place_msgs/action/ExecuteGraspMove_GetResult";
}

template<>
struct has_fixed_size<pick_and_place_msgs::action::ExecuteGraspMove_GetResult>
  : std::integral_constant<
    bool,
    has_fixed_size<pick_and_place_msgs::action::ExecuteGraspMove_GetResult_Request>::value &&
    has_fixed_size<pick_and_place_msgs::action::ExecuteGraspMove_GetResult_Response>::value
  >
{
};

template<>
struct has_bounded_size<pick_and_place_msgs::action::ExecuteGraspMove_GetResult>
  : std::integral_constant<
    bool,
    has_bounded_size<pick_and_place_msgs::action::ExecuteGraspMove_GetResult_Request>::value &&
    has_bounded_size<pick_and_place_msgs::action::ExecuteGraspMove_GetResult_Response>::value
  >
{
};

template<>
struct is_service<pick_and_place_msgs::action::ExecuteGraspMove_GetResult>
  : std::true_type
{
};

template<>
struct is_service_request<pick_and_place_msgs::action::ExecuteGraspMove_GetResult_Request>
  : std::true_type
{
};

template<>
struct is_service_response<pick_and_place_msgs::action::ExecuteGraspMove_GetResult_Response>
  : std::true_type
{
};

}  // namespace rosidl_generator_traits

// Include directives for member types
// Member 'goal_id'
// already included above
// #include "unique_identifier_msgs/msg/detail/uuid__traits.hpp"
// Member 'feedback'
// already included above
// #include "pick_and_place_msgs/action/detail/execute_grasp_move__traits.hpp"

namespace pick_and_place_msgs
{

namespace action
{

inline void to_flow_style_yaml(
  const ExecuteGraspMove_FeedbackMessage & msg,
  std::ostream & out)
{
  out << "{";
  // member: goal_id
  {
    out << "goal_id: ";
    to_flow_style_yaml(msg.goal_id, out);
    out << ", ";
  }

  // member: feedback
  {
    out << "feedback: ";
    to_flow_style_yaml(msg.feedback, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const ExecuteGraspMove_FeedbackMessage & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: goal_id
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "goal_id:\n";
    to_block_style_yaml(msg.goal_id, out, indentation + 2);
  }

  // member: feedback
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "feedback:\n";
    to_block_style_yaml(msg.feedback, out, indentation + 2);
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const ExecuteGraspMove_FeedbackMessage & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace action

}  // namespace pick_and_place_msgs

namespace rosidl_generator_traits
{

[[deprecated("use pick_and_place_msgs::action::to_block_style_yaml() instead")]]
inline void to_yaml(
  const pick_and_place_msgs::action::ExecuteGraspMove_FeedbackMessage & msg,
  std::ostream & out, size_t indentation = 0)
{
  pick_and_place_msgs::action::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use pick_and_place_msgs::action::to_yaml() instead")]]
inline std::string to_yaml(const pick_and_place_msgs::action::ExecuteGraspMove_FeedbackMessage & msg)
{
  return pick_and_place_msgs::action::to_yaml(msg);
}

template<>
inline const char * data_type<pick_and_place_msgs::action::ExecuteGraspMove_FeedbackMessage>()
{
  return "pick_and_place_msgs::action::ExecuteGraspMove_FeedbackMessage";
}

template<>
inline const char * name<pick_and_place_msgs::action::ExecuteGraspMove_FeedbackMessage>()
{
  return "pick_and_place_msgs/action/ExecuteGraspMove_FeedbackMessage";
}

template<>
struct has_fixed_size<pick_and_place_msgs::action::ExecuteGraspMove_FeedbackMessage>
  : std::integral_constant<bool, has_fixed_size<pick_and_place_msgs::action::ExecuteGraspMove_Feedback>::value && has_fixed_size<unique_identifier_msgs::msg::UUID>::value> {};

template<>
struct has_bounded_size<pick_and_place_msgs::action::ExecuteGraspMove_FeedbackMessage>
  : std::integral_constant<bool, has_bounded_size<pick_and_place_msgs::action::ExecuteGraspMove_Feedback>::value && has_bounded_size<unique_identifier_msgs::msg::UUID>::value> {};

template<>
struct is_message<pick_and_place_msgs::action::ExecuteGraspMove_FeedbackMessage>
  : std::true_type {};

}  // namespace rosidl_generator_traits


namespace rosidl_generator_traits
{

template<>
struct is_action<pick_and_place_msgs::action::ExecuteGraspMove>
  : std::true_type
{
};

template<>
struct is_action_goal<pick_and_place_msgs::action::ExecuteGraspMove_Goal>
  : std::true_type
{
};

template<>
struct is_action_result<pick_and_place_msgs::action::ExecuteGraspMove_Result>
  : std::true_type
{
};

template<>
struct is_action_feedback<pick_and_place_msgs::action::ExecuteGraspMove_Feedback>
  : std::true_type
{
};

}  // namespace rosidl_generator_traits


#endif  // PICK_AND_PLACE_MSGS__ACTION__DETAIL__EXECUTE_GRASP_MOVE__TRAITS_HPP_
