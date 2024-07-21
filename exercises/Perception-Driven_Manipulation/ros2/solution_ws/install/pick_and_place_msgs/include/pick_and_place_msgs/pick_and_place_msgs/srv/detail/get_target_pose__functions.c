// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from pick_and_place_msgs:srv/GetTargetPose.idl
// generated code does not contain a copyright notice
#include "pick_and_place_msgs/srv/detail/get_target_pose__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"

// Include directives for member types
// Member `world_frame_id`
// Member `ar_tag_frame_id`
#include "rosidl_runtime_c/string_functions.h"
// Member `shape`
#include "shape_msgs/msg/detail/solid_primitive__functions.h"
// Member `remove_at_poses`
#include "geometry_msgs/msg/detail/pose__functions.h"

bool
pick_and_place_msgs__srv__GetTargetPose_Request__init(pick_and_place_msgs__srv__GetTargetPose_Request * msg)
{
  if (!msg) {
    return false;
  }
  // world_frame_id
  if (!rosidl_runtime_c__String__init(&msg->world_frame_id)) {
    pick_and_place_msgs__srv__GetTargetPose_Request__fini(msg);
    return false;
  }
  // ar_tag_frame_id
  if (!rosidl_runtime_c__String__init(&msg->ar_tag_frame_id)) {
    pick_and_place_msgs__srv__GetTargetPose_Request__fini(msg);
    return false;
  }
  // shape
  if (!shape_msgs__msg__SolidPrimitive__init(&msg->shape)) {
    pick_and_place_msgs__srv__GetTargetPose_Request__fini(msg);
    return false;
  }
  // remove_at_poses
  if (!geometry_msgs__msg__Pose__Sequence__init(&msg->remove_at_poses, 0)) {
    pick_and_place_msgs__srv__GetTargetPose_Request__fini(msg);
    return false;
  }
  return true;
}

void
pick_and_place_msgs__srv__GetTargetPose_Request__fini(pick_and_place_msgs__srv__GetTargetPose_Request * msg)
{
  if (!msg) {
    return;
  }
  // world_frame_id
  rosidl_runtime_c__String__fini(&msg->world_frame_id);
  // ar_tag_frame_id
  rosidl_runtime_c__String__fini(&msg->ar_tag_frame_id);
  // shape
  shape_msgs__msg__SolidPrimitive__fini(&msg->shape);
  // remove_at_poses
  geometry_msgs__msg__Pose__Sequence__fini(&msg->remove_at_poses);
}

bool
pick_and_place_msgs__srv__GetTargetPose_Request__are_equal(const pick_and_place_msgs__srv__GetTargetPose_Request * lhs, const pick_and_place_msgs__srv__GetTargetPose_Request * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // world_frame_id
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->world_frame_id), &(rhs->world_frame_id)))
  {
    return false;
  }
  // ar_tag_frame_id
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->ar_tag_frame_id), &(rhs->ar_tag_frame_id)))
  {
    return false;
  }
  // shape
  if (!shape_msgs__msg__SolidPrimitive__are_equal(
      &(lhs->shape), &(rhs->shape)))
  {
    return false;
  }
  // remove_at_poses
  if (!geometry_msgs__msg__Pose__Sequence__are_equal(
      &(lhs->remove_at_poses), &(rhs->remove_at_poses)))
  {
    return false;
  }
  return true;
}

bool
pick_and_place_msgs__srv__GetTargetPose_Request__copy(
  const pick_and_place_msgs__srv__GetTargetPose_Request * input,
  pick_and_place_msgs__srv__GetTargetPose_Request * output)
{
  if (!input || !output) {
    return false;
  }
  // world_frame_id
  if (!rosidl_runtime_c__String__copy(
      &(input->world_frame_id), &(output->world_frame_id)))
  {
    return false;
  }
  // ar_tag_frame_id
  if (!rosidl_runtime_c__String__copy(
      &(input->ar_tag_frame_id), &(output->ar_tag_frame_id)))
  {
    return false;
  }
  // shape
  if (!shape_msgs__msg__SolidPrimitive__copy(
      &(input->shape), &(output->shape)))
  {
    return false;
  }
  // remove_at_poses
  if (!geometry_msgs__msg__Pose__Sequence__copy(
      &(input->remove_at_poses), &(output->remove_at_poses)))
  {
    return false;
  }
  return true;
}

pick_and_place_msgs__srv__GetTargetPose_Request *
pick_and_place_msgs__srv__GetTargetPose_Request__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  pick_and_place_msgs__srv__GetTargetPose_Request * msg = (pick_and_place_msgs__srv__GetTargetPose_Request *)allocator.allocate(sizeof(pick_and_place_msgs__srv__GetTargetPose_Request), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(pick_and_place_msgs__srv__GetTargetPose_Request));
  bool success = pick_and_place_msgs__srv__GetTargetPose_Request__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
pick_and_place_msgs__srv__GetTargetPose_Request__destroy(pick_and_place_msgs__srv__GetTargetPose_Request * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    pick_and_place_msgs__srv__GetTargetPose_Request__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
pick_and_place_msgs__srv__GetTargetPose_Request__Sequence__init(pick_and_place_msgs__srv__GetTargetPose_Request__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  pick_and_place_msgs__srv__GetTargetPose_Request * data = NULL;

  if (size) {
    data = (pick_and_place_msgs__srv__GetTargetPose_Request *)allocator.zero_allocate(size, sizeof(pick_and_place_msgs__srv__GetTargetPose_Request), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = pick_and_place_msgs__srv__GetTargetPose_Request__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        pick_and_place_msgs__srv__GetTargetPose_Request__fini(&data[i - 1]);
      }
      allocator.deallocate(data, allocator.state);
      return false;
    }
  }
  array->data = data;
  array->size = size;
  array->capacity = size;
  return true;
}

void
pick_and_place_msgs__srv__GetTargetPose_Request__Sequence__fini(pick_and_place_msgs__srv__GetTargetPose_Request__Sequence * array)
{
  if (!array) {
    return;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();

  if (array->data) {
    // ensure that data and capacity values are consistent
    assert(array->capacity > 0);
    // finalize all array elements
    for (size_t i = 0; i < array->capacity; ++i) {
      pick_and_place_msgs__srv__GetTargetPose_Request__fini(&array->data[i]);
    }
    allocator.deallocate(array->data, allocator.state);
    array->data = NULL;
    array->size = 0;
    array->capacity = 0;
  } else {
    // ensure that data, size, and capacity values are consistent
    assert(0 == array->size);
    assert(0 == array->capacity);
  }
}

pick_and_place_msgs__srv__GetTargetPose_Request__Sequence *
pick_and_place_msgs__srv__GetTargetPose_Request__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  pick_and_place_msgs__srv__GetTargetPose_Request__Sequence * array = (pick_and_place_msgs__srv__GetTargetPose_Request__Sequence *)allocator.allocate(sizeof(pick_and_place_msgs__srv__GetTargetPose_Request__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = pick_and_place_msgs__srv__GetTargetPose_Request__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
pick_and_place_msgs__srv__GetTargetPose_Request__Sequence__destroy(pick_and_place_msgs__srv__GetTargetPose_Request__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    pick_and_place_msgs__srv__GetTargetPose_Request__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
pick_and_place_msgs__srv__GetTargetPose_Request__Sequence__are_equal(const pick_and_place_msgs__srv__GetTargetPose_Request__Sequence * lhs, const pick_and_place_msgs__srv__GetTargetPose_Request__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!pick_and_place_msgs__srv__GetTargetPose_Request__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
pick_and_place_msgs__srv__GetTargetPose_Request__Sequence__copy(
  const pick_and_place_msgs__srv__GetTargetPose_Request__Sequence * input,
  pick_and_place_msgs__srv__GetTargetPose_Request__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(pick_and_place_msgs__srv__GetTargetPose_Request);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    pick_and_place_msgs__srv__GetTargetPose_Request * data =
      (pick_and_place_msgs__srv__GetTargetPose_Request *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!pick_and_place_msgs__srv__GetTargetPose_Request__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          pick_and_place_msgs__srv__GetTargetPose_Request__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!pick_and_place_msgs__srv__GetTargetPose_Request__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}


// Include directives for member types
// Member `target_pose`
// already included above
// #include "geometry_msgs/msg/detail/pose__functions.h"

bool
pick_and_place_msgs__srv__GetTargetPose_Response__init(pick_and_place_msgs__srv__GetTargetPose_Response * msg)
{
  if (!msg) {
    return false;
  }
  // succeeded
  // target_pose
  if (!geometry_msgs__msg__Pose__init(&msg->target_pose)) {
    pick_and_place_msgs__srv__GetTargetPose_Response__fini(msg);
    return false;
  }
  return true;
}

void
pick_and_place_msgs__srv__GetTargetPose_Response__fini(pick_and_place_msgs__srv__GetTargetPose_Response * msg)
{
  if (!msg) {
    return;
  }
  // succeeded
  // target_pose
  geometry_msgs__msg__Pose__fini(&msg->target_pose);
}

bool
pick_and_place_msgs__srv__GetTargetPose_Response__are_equal(const pick_and_place_msgs__srv__GetTargetPose_Response * lhs, const pick_and_place_msgs__srv__GetTargetPose_Response * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // succeeded
  if (lhs->succeeded != rhs->succeeded) {
    return false;
  }
  // target_pose
  if (!geometry_msgs__msg__Pose__are_equal(
      &(lhs->target_pose), &(rhs->target_pose)))
  {
    return false;
  }
  return true;
}

bool
pick_and_place_msgs__srv__GetTargetPose_Response__copy(
  const pick_and_place_msgs__srv__GetTargetPose_Response * input,
  pick_and_place_msgs__srv__GetTargetPose_Response * output)
{
  if (!input || !output) {
    return false;
  }
  // succeeded
  output->succeeded = input->succeeded;
  // target_pose
  if (!geometry_msgs__msg__Pose__copy(
      &(input->target_pose), &(output->target_pose)))
  {
    return false;
  }
  return true;
}

pick_and_place_msgs__srv__GetTargetPose_Response *
pick_and_place_msgs__srv__GetTargetPose_Response__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  pick_and_place_msgs__srv__GetTargetPose_Response * msg = (pick_and_place_msgs__srv__GetTargetPose_Response *)allocator.allocate(sizeof(pick_and_place_msgs__srv__GetTargetPose_Response), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(pick_and_place_msgs__srv__GetTargetPose_Response));
  bool success = pick_and_place_msgs__srv__GetTargetPose_Response__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
pick_and_place_msgs__srv__GetTargetPose_Response__destroy(pick_and_place_msgs__srv__GetTargetPose_Response * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    pick_and_place_msgs__srv__GetTargetPose_Response__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
pick_and_place_msgs__srv__GetTargetPose_Response__Sequence__init(pick_and_place_msgs__srv__GetTargetPose_Response__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  pick_and_place_msgs__srv__GetTargetPose_Response * data = NULL;

  if (size) {
    data = (pick_and_place_msgs__srv__GetTargetPose_Response *)allocator.zero_allocate(size, sizeof(pick_and_place_msgs__srv__GetTargetPose_Response), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = pick_and_place_msgs__srv__GetTargetPose_Response__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        pick_and_place_msgs__srv__GetTargetPose_Response__fini(&data[i - 1]);
      }
      allocator.deallocate(data, allocator.state);
      return false;
    }
  }
  array->data = data;
  array->size = size;
  array->capacity = size;
  return true;
}

void
pick_and_place_msgs__srv__GetTargetPose_Response__Sequence__fini(pick_and_place_msgs__srv__GetTargetPose_Response__Sequence * array)
{
  if (!array) {
    return;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();

  if (array->data) {
    // ensure that data and capacity values are consistent
    assert(array->capacity > 0);
    // finalize all array elements
    for (size_t i = 0; i < array->capacity; ++i) {
      pick_and_place_msgs__srv__GetTargetPose_Response__fini(&array->data[i]);
    }
    allocator.deallocate(array->data, allocator.state);
    array->data = NULL;
    array->size = 0;
    array->capacity = 0;
  } else {
    // ensure that data, size, and capacity values are consistent
    assert(0 == array->size);
    assert(0 == array->capacity);
  }
}

pick_and_place_msgs__srv__GetTargetPose_Response__Sequence *
pick_and_place_msgs__srv__GetTargetPose_Response__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  pick_and_place_msgs__srv__GetTargetPose_Response__Sequence * array = (pick_and_place_msgs__srv__GetTargetPose_Response__Sequence *)allocator.allocate(sizeof(pick_and_place_msgs__srv__GetTargetPose_Response__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = pick_and_place_msgs__srv__GetTargetPose_Response__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
pick_and_place_msgs__srv__GetTargetPose_Response__Sequence__destroy(pick_and_place_msgs__srv__GetTargetPose_Response__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    pick_and_place_msgs__srv__GetTargetPose_Response__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
pick_and_place_msgs__srv__GetTargetPose_Response__Sequence__are_equal(const pick_and_place_msgs__srv__GetTargetPose_Response__Sequence * lhs, const pick_and_place_msgs__srv__GetTargetPose_Response__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!pick_and_place_msgs__srv__GetTargetPose_Response__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
pick_and_place_msgs__srv__GetTargetPose_Response__Sequence__copy(
  const pick_and_place_msgs__srv__GetTargetPose_Response__Sequence * input,
  pick_and_place_msgs__srv__GetTargetPose_Response__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(pick_and_place_msgs__srv__GetTargetPose_Response);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    pick_and_place_msgs__srv__GetTargetPose_Response * data =
      (pick_and_place_msgs__srv__GetTargetPose_Response *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!pick_and_place_msgs__srv__GetTargetPose_Response__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          pick_and_place_msgs__srv__GetTargetPose_Response__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!pick_and_place_msgs__srv__GetTargetPose_Response__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
