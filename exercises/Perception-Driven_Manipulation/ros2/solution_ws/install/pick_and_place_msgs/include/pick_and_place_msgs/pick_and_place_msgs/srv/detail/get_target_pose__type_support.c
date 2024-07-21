// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from pick_and_place_msgs:srv/GetTargetPose.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "pick_and_place_msgs/srv/detail/get_target_pose__rosidl_typesupport_introspection_c.h"
#include "pick_and_place_msgs/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "pick_and_place_msgs/srv/detail/get_target_pose__functions.h"
#include "pick_and_place_msgs/srv/detail/get_target_pose__struct.h"


// Include directives for member types
// Member `world_frame_id`
// Member `ar_tag_frame_id`
#include "rosidl_runtime_c/string_functions.h"
// Member `shape`
#include "shape_msgs/msg/solid_primitive.h"
// Member `shape`
#include "shape_msgs/msg/detail/solid_primitive__rosidl_typesupport_introspection_c.h"
// Member `remove_at_poses`
#include "geometry_msgs/msg/pose.h"
// Member `remove_at_poses`
#include "geometry_msgs/msg/detail/pose__rosidl_typesupport_introspection_c.h"

#ifdef __cplusplus
extern "C"
{
#endif

void pick_and_place_msgs__srv__GetTargetPose_Request__rosidl_typesupport_introspection_c__GetTargetPose_Request_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  pick_and_place_msgs__srv__GetTargetPose_Request__init(message_memory);
}

void pick_and_place_msgs__srv__GetTargetPose_Request__rosidl_typesupport_introspection_c__GetTargetPose_Request_fini_function(void * message_memory)
{
  pick_and_place_msgs__srv__GetTargetPose_Request__fini(message_memory);
}

size_t pick_and_place_msgs__srv__GetTargetPose_Request__rosidl_typesupport_introspection_c__size_function__GetTargetPose_Request__remove_at_poses(
  const void * untyped_member)
{
  const geometry_msgs__msg__Pose__Sequence * member =
    (const geometry_msgs__msg__Pose__Sequence *)(untyped_member);
  return member->size;
}

const void * pick_and_place_msgs__srv__GetTargetPose_Request__rosidl_typesupport_introspection_c__get_const_function__GetTargetPose_Request__remove_at_poses(
  const void * untyped_member, size_t index)
{
  const geometry_msgs__msg__Pose__Sequence * member =
    (const geometry_msgs__msg__Pose__Sequence *)(untyped_member);
  return &member->data[index];
}

void * pick_and_place_msgs__srv__GetTargetPose_Request__rosidl_typesupport_introspection_c__get_function__GetTargetPose_Request__remove_at_poses(
  void * untyped_member, size_t index)
{
  geometry_msgs__msg__Pose__Sequence * member =
    (geometry_msgs__msg__Pose__Sequence *)(untyped_member);
  return &member->data[index];
}

void pick_and_place_msgs__srv__GetTargetPose_Request__rosidl_typesupport_introspection_c__fetch_function__GetTargetPose_Request__remove_at_poses(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const geometry_msgs__msg__Pose * item =
    ((const geometry_msgs__msg__Pose *)
    pick_and_place_msgs__srv__GetTargetPose_Request__rosidl_typesupport_introspection_c__get_const_function__GetTargetPose_Request__remove_at_poses(untyped_member, index));
  geometry_msgs__msg__Pose * value =
    (geometry_msgs__msg__Pose *)(untyped_value);
  *value = *item;
}

void pick_and_place_msgs__srv__GetTargetPose_Request__rosidl_typesupport_introspection_c__assign_function__GetTargetPose_Request__remove_at_poses(
  void * untyped_member, size_t index, const void * untyped_value)
{
  geometry_msgs__msg__Pose * item =
    ((geometry_msgs__msg__Pose *)
    pick_and_place_msgs__srv__GetTargetPose_Request__rosidl_typesupport_introspection_c__get_function__GetTargetPose_Request__remove_at_poses(untyped_member, index));
  const geometry_msgs__msg__Pose * value =
    (const geometry_msgs__msg__Pose *)(untyped_value);
  *item = *value;
}

bool pick_and_place_msgs__srv__GetTargetPose_Request__rosidl_typesupport_introspection_c__resize_function__GetTargetPose_Request__remove_at_poses(
  void * untyped_member, size_t size)
{
  geometry_msgs__msg__Pose__Sequence * member =
    (geometry_msgs__msg__Pose__Sequence *)(untyped_member);
  geometry_msgs__msg__Pose__Sequence__fini(member);
  return geometry_msgs__msg__Pose__Sequence__init(member, size);
}

static rosidl_typesupport_introspection_c__MessageMember pick_and_place_msgs__srv__GetTargetPose_Request__rosidl_typesupport_introspection_c__GetTargetPose_Request_message_member_array[4] = {
  {
    "world_frame_id",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_STRING,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(pick_and_place_msgs__srv__GetTargetPose_Request, world_frame_id),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "ar_tag_frame_id",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_STRING,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(pick_and_place_msgs__srv__GetTargetPose_Request, ar_tag_frame_id),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "shape",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(pick_and_place_msgs__srv__GetTargetPose_Request, shape),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "remove_at_poses",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(pick_and_place_msgs__srv__GetTargetPose_Request, remove_at_poses),  // bytes offset in struct
    NULL,  // default value
    pick_and_place_msgs__srv__GetTargetPose_Request__rosidl_typesupport_introspection_c__size_function__GetTargetPose_Request__remove_at_poses,  // size() function pointer
    pick_and_place_msgs__srv__GetTargetPose_Request__rosidl_typesupport_introspection_c__get_const_function__GetTargetPose_Request__remove_at_poses,  // get_const(index) function pointer
    pick_and_place_msgs__srv__GetTargetPose_Request__rosidl_typesupport_introspection_c__get_function__GetTargetPose_Request__remove_at_poses,  // get(index) function pointer
    pick_and_place_msgs__srv__GetTargetPose_Request__rosidl_typesupport_introspection_c__fetch_function__GetTargetPose_Request__remove_at_poses,  // fetch(index, &value) function pointer
    pick_and_place_msgs__srv__GetTargetPose_Request__rosidl_typesupport_introspection_c__assign_function__GetTargetPose_Request__remove_at_poses,  // assign(index, value) function pointer
    pick_and_place_msgs__srv__GetTargetPose_Request__rosidl_typesupport_introspection_c__resize_function__GetTargetPose_Request__remove_at_poses  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers pick_and_place_msgs__srv__GetTargetPose_Request__rosidl_typesupport_introspection_c__GetTargetPose_Request_message_members = {
  "pick_and_place_msgs__srv",  // message namespace
  "GetTargetPose_Request",  // message name
  4,  // number of fields
  sizeof(pick_and_place_msgs__srv__GetTargetPose_Request),
  pick_and_place_msgs__srv__GetTargetPose_Request__rosidl_typesupport_introspection_c__GetTargetPose_Request_message_member_array,  // message members
  pick_and_place_msgs__srv__GetTargetPose_Request__rosidl_typesupport_introspection_c__GetTargetPose_Request_init_function,  // function to initialize message memory (memory has to be allocated)
  pick_and_place_msgs__srv__GetTargetPose_Request__rosidl_typesupport_introspection_c__GetTargetPose_Request_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t pick_and_place_msgs__srv__GetTargetPose_Request__rosidl_typesupport_introspection_c__GetTargetPose_Request_message_type_support_handle = {
  0,
  &pick_and_place_msgs__srv__GetTargetPose_Request__rosidl_typesupport_introspection_c__GetTargetPose_Request_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_pick_and_place_msgs
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, pick_and_place_msgs, srv, GetTargetPose_Request)() {
  pick_and_place_msgs__srv__GetTargetPose_Request__rosidl_typesupport_introspection_c__GetTargetPose_Request_message_member_array[2].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, shape_msgs, msg, SolidPrimitive)();
  pick_and_place_msgs__srv__GetTargetPose_Request__rosidl_typesupport_introspection_c__GetTargetPose_Request_message_member_array[3].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, geometry_msgs, msg, Pose)();
  if (!pick_and_place_msgs__srv__GetTargetPose_Request__rosidl_typesupport_introspection_c__GetTargetPose_Request_message_type_support_handle.typesupport_identifier) {
    pick_and_place_msgs__srv__GetTargetPose_Request__rosidl_typesupport_introspection_c__GetTargetPose_Request_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &pick_and_place_msgs__srv__GetTargetPose_Request__rosidl_typesupport_introspection_c__GetTargetPose_Request_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif

// already included above
// #include <stddef.h>
// already included above
// #include "pick_and_place_msgs/srv/detail/get_target_pose__rosidl_typesupport_introspection_c.h"
// already included above
// #include "pick_and_place_msgs/msg/rosidl_typesupport_introspection_c__visibility_control.h"
// already included above
// #include "rosidl_typesupport_introspection_c/field_types.h"
// already included above
// #include "rosidl_typesupport_introspection_c/identifier.h"
// already included above
// #include "rosidl_typesupport_introspection_c/message_introspection.h"
// already included above
// #include "pick_and_place_msgs/srv/detail/get_target_pose__functions.h"
// already included above
// #include "pick_and_place_msgs/srv/detail/get_target_pose__struct.h"


// Include directives for member types
// Member `target_pose`
// already included above
// #include "geometry_msgs/msg/pose.h"
// Member `target_pose`
// already included above
// #include "geometry_msgs/msg/detail/pose__rosidl_typesupport_introspection_c.h"

#ifdef __cplusplus
extern "C"
{
#endif

void pick_and_place_msgs__srv__GetTargetPose_Response__rosidl_typesupport_introspection_c__GetTargetPose_Response_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  pick_and_place_msgs__srv__GetTargetPose_Response__init(message_memory);
}

void pick_and_place_msgs__srv__GetTargetPose_Response__rosidl_typesupport_introspection_c__GetTargetPose_Response_fini_function(void * message_memory)
{
  pick_and_place_msgs__srv__GetTargetPose_Response__fini(message_memory);
}

static rosidl_typesupport_introspection_c__MessageMember pick_and_place_msgs__srv__GetTargetPose_Response__rosidl_typesupport_introspection_c__GetTargetPose_Response_message_member_array[2] = {
  {
    "succeeded",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_BOOLEAN,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(pick_and_place_msgs__srv__GetTargetPose_Response, succeeded),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "target_pose",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(pick_and_place_msgs__srv__GetTargetPose_Response, target_pose),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers pick_and_place_msgs__srv__GetTargetPose_Response__rosidl_typesupport_introspection_c__GetTargetPose_Response_message_members = {
  "pick_and_place_msgs__srv",  // message namespace
  "GetTargetPose_Response",  // message name
  2,  // number of fields
  sizeof(pick_and_place_msgs__srv__GetTargetPose_Response),
  pick_and_place_msgs__srv__GetTargetPose_Response__rosidl_typesupport_introspection_c__GetTargetPose_Response_message_member_array,  // message members
  pick_and_place_msgs__srv__GetTargetPose_Response__rosidl_typesupport_introspection_c__GetTargetPose_Response_init_function,  // function to initialize message memory (memory has to be allocated)
  pick_and_place_msgs__srv__GetTargetPose_Response__rosidl_typesupport_introspection_c__GetTargetPose_Response_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t pick_and_place_msgs__srv__GetTargetPose_Response__rosidl_typesupport_introspection_c__GetTargetPose_Response_message_type_support_handle = {
  0,
  &pick_and_place_msgs__srv__GetTargetPose_Response__rosidl_typesupport_introspection_c__GetTargetPose_Response_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_pick_and_place_msgs
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, pick_and_place_msgs, srv, GetTargetPose_Response)() {
  pick_and_place_msgs__srv__GetTargetPose_Response__rosidl_typesupport_introspection_c__GetTargetPose_Response_message_member_array[1].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, geometry_msgs, msg, Pose)();
  if (!pick_and_place_msgs__srv__GetTargetPose_Response__rosidl_typesupport_introspection_c__GetTargetPose_Response_message_type_support_handle.typesupport_identifier) {
    pick_and_place_msgs__srv__GetTargetPose_Response__rosidl_typesupport_introspection_c__GetTargetPose_Response_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &pick_and_place_msgs__srv__GetTargetPose_Response__rosidl_typesupport_introspection_c__GetTargetPose_Response_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif

#include "rosidl_runtime_c/service_type_support_struct.h"
// already included above
// #include "pick_and_place_msgs/msg/rosidl_typesupport_introspection_c__visibility_control.h"
// already included above
// #include "pick_and_place_msgs/srv/detail/get_target_pose__rosidl_typesupport_introspection_c.h"
// already included above
// #include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/service_introspection.h"

// this is intentionally not const to allow initialization later to prevent an initialization race
static rosidl_typesupport_introspection_c__ServiceMembers pick_and_place_msgs__srv__detail__get_target_pose__rosidl_typesupport_introspection_c__GetTargetPose_service_members = {
  "pick_and_place_msgs__srv",  // service namespace
  "GetTargetPose",  // service name
  // these two fields are initialized below on the first access
  NULL,  // request message
  // pick_and_place_msgs__srv__detail__get_target_pose__rosidl_typesupport_introspection_c__GetTargetPose_Request_message_type_support_handle,
  NULL  // response message
  // pick_and_place_msgs__srv__detail__get_target_pose__rosidl_typesupport_introspection_c__GetTargetPose_Response_message_type_support_handle
};

static rosidl_service_type_support_t pick_and_place_msgs__srv__detail__get_target_pose__rosidl_typesupport_introspection_c__GetTargetPose_service_type_support_handle = {
  0,
  &pick_and_place_msgs__srv__detail__get_target_pose__rosidl_typesupport_introspection_c__GetTargetPose_service_members,
  get_service_typesupport_handle_function,
};

// Forward declaration of request/response type support functions
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, pick_and_place_msgs, srv, GetTargetPose_Request)();

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, pick_and_place_msgs, srv, GetTargetPose_Response)();

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_pick_and_place_msgs
const rosidl_service_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_introspection_c, pick_and_place_msgs, srv, GetTargetPose)() {
  if (!pick_and_place_msgs__srv__detail__get_target_pose__rosidl_typesupport_introspection_c__GetTargetPose_service_type_support_handle.typesupport_identifier) {
    pick_and_place_msgs__srv__detail__get_target_pose__rosidl_typesupport_introspection_c__GetTargetPose_service_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  rosidl_typesupport_introspection_c__ServiceMembers * service_members =
    (rosidl_typesupport_introspection_c__ServiceMembers *)pick_and_place_msgs__srv__detail__get_target_pose__rosidl_typesupport_introspection_c__GetTargetPose_service_type_support_handle.data;

  if (!service_members->request_members_) {
    service_members->request_members_ =
      (const rosidl_typesupport_introspection_c__MessageMembers *)
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, pick_and_place_msgs, srv, GetTargetPose_Request)()->data;
  }
  if (!service_members->response_members_) {
    service_members->response_members_ =
      (const rosidl_typesupport_introspection_c__MessageMembers *)
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, pick_and_place_msgs, srv, GetTargetPose_Response)()->data;
  }

  return &pick_and_place_msgs__srv__detail__get_target_pose__rosidl_typesupport_introspection_c__GetTargetPose_service_type_support_handle;
}
