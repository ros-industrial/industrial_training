// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from pick_and_place_msgs:srv/GetTargetPose.idl
// generated code does not contain a copyright notice

#ifndef PICK_AND_PLACE_MSGS__SRV__DETAIL__GET_TARGET_POSE__STRUCT_H_
#define PICK_AND_PLACE_MSGS__SRV__DETAIL__GET_TARGET_POSE__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'world_frame_id'
// Member 'ar_tag_frame_id'
#include "rosidl_runtime_c/string.h"
// Member 'shape'
#include "shape_msgs/msg/detail/solid_primitive__struct.h"
// Member 'remove_at_poses'
#include "geometry_msgs/msg/detail/pose__struct.h"

/// Struct defined in srv/GetTargetPose in the package pick_and_place_msgs.
typedef struct pick_and_place_msgs__srv__GetTargetPose_Request
{
  rosidl_runtime_c__String world_frame_id;
  rosidl_runtime_c__String ar_tag_frame_id;
  shape_msgs__msg__SolidPrimitive shape;
  geometry_msgs__msg__Pose__Sequence remove_at_poses;
} pick_and_place_msgs__srv__GetTargetPose_Request;

// Struct for a sequence of pick_and_place_msgs__srv__GetTargetPose_Request.
typedef struct pick_and_place_msgs__srv__GetTargetPose_Request__Sequence
{
  pick_and_place_msgs__srv__GetTargetPose_Request * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} pick_and_place_msgs__srv__GetTargetPose_Request__Sequence;


// Constants defined in the message

// Include directives for member types
// Member 'target_pose'
// already included above
// #include "geometry_msgs/msg/detail/pose__struct.h"

/// Struct defined in srv/GetTargetPose in the package pick_and_place_msgs.
typedef struct pick_and_place_msgs__srv__GetTargetPose_Response
{
  bool succeeded;
  geometry_msgs__msg__Pose target_pose;
} pick_and_place_msgs__srv__GetTargetPose_Response;

// Struct for a sequence of pick_and_place_msgs__srv__GetTargetPose_Response.
typedef struct pick_and_place_msgs__srv__GetTargetPose_Response__Sequence
{
  pick_and_place_msgs__srv__GetTargetPose_Response * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} pick_and_place_msgs__srv__GetTargetPose_Response__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // PICK_AND_PLACE_MSGS__SRV__DETAIL__GET_TARGET_POSE__STRUCT_H_
