// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from pick_and_place_msgs:action/ExecuteGraspMove.idl
// generated code does not contain a copyright notice
#include "pick_and_place_msgs/action/detail/execute_grasp_move__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


bool
pick_and_place_msgs__action__ExecuteGraspMove_Goal__init(pick_and_place_msgs__action__ExecuteGraspMove_Goal * msg)
{
  if (!msg) {
    return false;
  }
  // goal
  // max_contact_force
  return true;
}

void
pick_and_place_msgs__action__ExecuteGraspMove_Goal__fini(pick_and_place_msgs__action__ExecuteGraspMove_Goal * msg)
{
  if (!msg) {
    return;
  }
  // goal
  // max_contact_force
}

bool
pick_and_place_msgs__action__ExecuteGraspMove_Goal__are_equal(const pick_and_place_msgs__action__ExecuteGraspMove_Goal * lhs, const pick_and_place_msgs__action__ExecuteGraspMove_Goal * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // goal
  if (lhs->goal != rhs->goal) {
    return false;
  }
  // max_contact_force
  if (lhs->max_contact_force != rhs->max_contact_force) {
    return false;
  }
  return true;
}

bool
pick_and_place_msgs__action__ExecuteGraspMove_Goal__copy(
  const pick_and_place_msgs__action__ExecuteGraspMove_Goal * input,
  pick_and_place_msgs__action__ExecuteGraspMove_Goal * output)
{
  if (!input || !output) {
    return false;
  }
  // goal
  output->goal = input->goal;
  // max_contact_force
  output->max_contact_force = input->max_contact_force;
  return true;
}

pick_and_place_msgs__action__ExecuteGraspMove_Goal *
pick_and_place_msgs__action__ExecuteGraspMove_Goal__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  pick_and_place_msgs__action__ExecuteGraspMove_Goal * msg = (pick_and_place_msgs__action__ExecuteGraspMove_Goal *)allocator.allocate(sizeof(pick_and_place_msgs__action__ExecuteGraspMove_Goal), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(pick_and_place_msgs__action__ExecuteGraspMove_Goal));
  bool success = pick_and_place_msgs__action__ExecuteGraspMove_Goal__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
pick_and_place_msgs__action__ExecuteGraspMove_Goal__destroy(pick_and_place_msgs__action__ExecuteGraspMove_Goal * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    pick_and_place_msgs__action__ExecuteGraspMove_Goal__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
pick_and_place_msgs__action__ExecuteGraspMove_Goal__Sequence__init(pick_and_place_msgs__action__ExecuteGraspMove_Goal__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  pick_and_place_msgs__action__ExecuteGraspMove_Goal * data = NULL;

  if (size) {
    data = (pick_and_place_msgs__action__ExecuteGraspMove_Goal *)allocator.zero_allocate(size, sizeof(pick_and_place_msgs__action__ExecuteGraspMove_Goal), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = pick_and_place_msgs__action__ExecuteGraspMove_Goal__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        pick_and_place_msgs__action__ExecuteGraspMove_Goal__fini(&data[i - 1]);
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
pick_and_place_msgs__action__ExecuteGraspMove_Goal__Sequence__fini(pick_and_place_msgs__action__ExecuteGraspMove_Goal__Sequence * array)
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
      pick_and_place_msgs__action__ExecuteGraspMove_Goal__fini(&array->data[i]);
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

pick_and_place_msgs__action__ExecuteGraspMove_Goal__Sequence *
pick_and_place_msgs__action__ExecuteGraspMove_Goal__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  pick_and_place_msgs__action__ExecuteGraspMove_Goal__Sequence * array = (pick_and_place_msgs__action__ExecuteGraspMove_Goal__Sequence *)allocator.allocate(sizeof(pick_and_place_msgs__action__ExecuteGraspMove_Goal__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = pick_and_place_msgs__action__ExecuteGraspMove_Goal__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
pick_and_place_msgs__action__ExecuteGraspMove_Goal__Sequence__destroy(pick_and_place_msgs__action__ExecuteGraspMove_Goal__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    pick_and_place_msgs__action__ExecuteGraspMove_Goal__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
pick_and_place_msgs__action__ExecuteGraspMove_Goal__Sequence__are_equal(const pick_and_place_msgs__action__ExecuteGraspMove_Goal__Sequence * lhs, const pick_and_place_msgs__action__ExecuteGraspMove_Goal__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!pick_and_place_msgs__action__ExecuteGraspMove_Goal__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
pick_and_place_msgs__action__ExecuteGraspMove_Goal__Sequence__copy(
  const pick_and_place_msgs__action__ExecuteGraspMove_Goal__Sequence * input,
  pick_and_place_msgs__action__ExecuteGraspMove_Goal__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(pick_and_place_msgs__action__ExecuteGraspMove_Goal);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    pick_and_place_msgs__action__ExecuteGraspMove_Goal * data =
      (pick_and_place_msgs__action__ExecuteGraspMove_Goal *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!pick_and_place_msgs__action__ExecuteGraspMove_Goal__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          pick_and_place_msgs__action__ExecuteGraspMove_Goal__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!pick_and_place_msgs__action__ExecuteGraspMove_Goal__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}


bool
pick_and_place_msgs__action__ExecuteGraspMove_Result__init(pick_and_place_msgs__action__ExecuteGraspMove_Result * msg)
{
  if (!msg) {
    return false;
  }
  // success
  return true;
}

void
pick_and_place_msgs__action__ExecuteGraspMove_Result__fini(pick_and_place_msgs__action__ExecuteGraspMove_Result * msg)
{
  if (!msg) {
    return;
  }
  // success
}

bool
pick_and_place_msgs__action__ExecuteGraspMove_Result__are_equal(const pick_and_place_msgs__action__ExecuteGraspMove_Result * lhs, const pick_and_place_msgs__action__ExecuteGraspMove_Result * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // success
  if (lhs->success != rhs->success) {
    return false;
  }
  return true;
}

bool
pick_and_place_msgs__action__ExecuteGraspMove_Result__copy(
  const pick_and_place_msgs__action__ExecuteGraspMove_Result * input,
  pick_and_place_msgs__action__ExecuteGraspMove_Result * output)
{
  if (!input || !output) {
    return false;
  }
  // success
  output->success = input->success;
  return true;
}

pick_and_place_msgs__action__ExecuteGraspMove_Result *
pick_and_place_msgs__action__ExecuteGraspMove_Result__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  pick_and_place_msgs__action__ExecuteGraspMove_Result * msg = (pick_and_place_msgs__action__ExecuteGraspMove_Result *)allocator.allocate(sizeof(pick_and_place_msgs__action__ExecuteGraspMove_Result), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(pick_and_place_msgs__action__ExecuteGraspMove_Result));
  bool success = pick_and_place_msgs__action__ExecuteGraspMove_Result__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
pick_and_place_msgs__action__ExecuteGraspMove_Result__destroy(pick_and_place_msgs__action__ExecuteGraspMove_Result * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    pick_and_place_msgs__action__ExecuteGraspMove_Result__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
pick_and_place_msgs__action__ExecuteGraspMove_Result__Sequence__init(pick_and_place_msgs__action__ExecuteGraspMove_Result__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  pick_and_place_msgs__action__ExecuteGraspMove_Result * data = NULL;

  if (size) {
    data = (pick_and_place_msgs__action__ExecuteGraspMove_Result *)allocator.zero_allocate(size, sizeof(pick_and_place_msgs__action__ExecuteGraspMove_Result), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = pick_and_place_msgs__action__ExecuteGraspMove_Result__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        pick_and_place_msgs__action__ExecuteGraspMove_Result__fini(&data[i - 1]);
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
pick_and_place_msgs__action__ExecuteGraspMove_Result__Sequence__fini(pick_and_place_msgs__action__ExecuteGraspMove_Result__Sequence * array)
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
      pick_and_place_msgs__action__ExecuteGraspMove_Result__fini(&array->data[i]);
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

pick_and_place_msgs__action__ExecuteGraspMove_Result__Sequence *
pick_and_place_msgs__action__ExecuteGraspMove_Result__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  pick_and_place_msgs__action__ExecuteGraspMove_Result__Sequence * array = (pick_and_place_msgs__action__ExecuteGraspMove_Result__Sequence *)allocator.allocate(sizeof(pick_and_place_msgs__action__ExecuteGraspMove_Result__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = pick_and_place_msgs__action__ExecuteGraspMove_Result__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
pick_and_place_msgs__action__ExecuteGraspMove_Result__Sequence__destroy(pick_and_place_msgs__action__ExecuteGraspMove_Result__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    pick_and_place_msgs__action__ExecuteGraspMove_Result__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
pick_and_place_msgs__action__ExecuteGraspMove_Result__Sequence__are_equal(const pick_and_place_msgs__action__ExecuteGraspMove_Result__Sequence * lhs, const pick_and_place_msgs__action__ExecuteGraspMove_Result__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!pick_and_place_msgs__action__ExecuteGraspMove_Result__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
pick_and_place_msgs__action__ExecuteGraspMove_Result__Sequence__copy(
  const pick_and_place_msgs__action__ExecuteGraspMove_Result__Sequence * input,
  pick_and_place_msgs__action__ExecuteGraspMove_Result__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(pick_and_place_msgs__action__ExecuteGraspMove_Result);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    pick_and_place_msgs__action__ExecuteGraspMove_Result * data =
      (pick_and_place_msgs__action__ExecuteGraspMove_Result *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!pick_and_place_msgs__action__ExecuteGraspMove_Result__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          pick_and_place_msgs__action__ExecuteGraspMove_Result__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!pick_and_place_msgs__action__ExecuteGraspMove_Result__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}


bool
pick_and_place_msgs__action__ExecuteGraspMove_Feedback__init(pick_and_place_msgs__action__ExecuteGraspMove_Feedback * msg)
{
  if (!msg) {
    return false;
  }
  // structure_needs_at_least_one_member
  return true;
}

void
pick_and_place_msgs__action__ExecuteGraspMove_Feedback__fini(pick_and_place_msgs__action__ExecuteGraspMove_Feedback * msg)
{
  if (!msg) {
    return;
  }
  // structure_needs_at_least_one_member
}

bool
pick_and_place_msgs__action__ExecuteGraspMove_Feedback__are_equal(const pick_and_place_msgs__action__ExecuteGraspMove_Feedback * lhs, const pick_and_place_msgs__action__ExecuteGraspMove_Feedback * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // structure_needs_at_least_one_member
  if (lhs->structure_needs_at_least_one_member != rhs->structure_needs_at_least_one_member) {
    return false;
  }
  return true;
}

bool
pick_and_place_msgs__action__ExecuteGraspMove_Feedback__copy(
  const pick_and_place_msgs__action__ExecuteGraspMove_Feedback * input,
  pick_and_place_msgs__action__ExecuteGraspMove_Feedback * output)
{
  if (!input || !output) {
    return false;
  }
  // structure_needs_at_least_one_member
  output->structure_needs_at_least_one_member = input->structure_needs_at_least_one_member;
  return true;
}

pick_and_place_msgs__action__ExecuteGraspMove_Feedback *
pick_and_place_msgs__action__ExecuteGraspMove_Feedback__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  pick_and_place_msgs__action__ExecuteGraspMove_Feedback * msg = (pick_and_place_msgs__action__ExecuteGraspMove_Feedback *)allocator.allocate(sizeof(pick_and_place_msgs__action__ExecuteGraspMove_Feedback), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(pick_and_place_msgs__action__ExecuteGraspMove_Feedback));
  bool success = pick_and_place_msgs__action__ExecuteGraspMove_Feedback__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
pick_and_place_msgs__action__ExecuteGraspMove_Feedback__destroy(pick_and_place_msgs__action__ExecuteGraspMove_Feedback * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    pick_and_place_msgs__action__ExecuteGraspMove_Feedback__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
pick_and_place_msgs__action__ExecuteGraspMove_Feedback__Sequence__init(pick_and_place_msgs__action__ExecuteGraspMove_Feedback__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  pick_and_place_msgs__action__ExecuteGraspMove_Feedback * data = NULL;

  if (size) {
    data = (pick_and_place_msgs__action__ExecuteGraspMove_Feedback *)allocator.zero_allocate(size, sizeof(pick_and_place_msgs__action__ExecuteGraspMove_Feedback), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = pick_and_place_msgs__action__ExecuteGraspMove_Feedback__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        pick_and_place_msgs__action__ExecuteGraspMove_Feedback__fini(&data[i - 1]);
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
pick_and_place_msgs__action__ExecuteGraspMove_Feedback__Sequence__fini(pick_and_place_msgs__action__ExecuteGraspMove_Feedback__Sequence * array)
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
      pick_and_place_msgs__action__ExecuteGraspMove_Feedback__fini(&array->data[i]);
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

pick_and_place_msgs__action__ExecuteGraspMove_Feedback__Sequence *
pick_and_place_msgs__action__ExecuteGraspMove_Feedback__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  pick_and_place_msgs__action__ExecuteGraspMove_Feedback__Sequence * array = (pick_and_place_msgs__action__ExecuteGraspMove_Feedback__Sequence *)allocator.allocate(sizeof(pick_and_place_msgs__action__ExecuteGraspMove_Feedback__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = pick_and_place_msgs__action__ExecuteGraspMove_Feedback__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
pick_and_place_msgs__action__ExecuteGraspMove_Feedback__Sequence__destroy(pick_and_place_msgs__action__ExecuteGraspMove_Feedback__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    pick_and_place_msgs__action__ExecuteGraspMove_Feedback__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
pick_and_place_msgs__action__ExecuteGraspMove_Feedback__Sequence__are_equal(const pick_and_place_msgs__action__ExecuteGraspMove_Feedback__Sequence * lhs, const pick_and_place_msgs__action__ExecuteGraspMove_Feedback__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!pick_and_place_msgs__action__ExecuteGraspMove_Feedback__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
pick_and_place_msgs__action__ExecuteGraspMove_Feedback__Sequence__copy(
  const pick_and_place_msgs__action__ExecuteGraspMove_Feedback__Sequence * input,
  pick_and_place_msgs__action__ExecuteGraspMove_Feedback__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(pick_and_place_msgs__action__ExecuteGraspMove_Feedback);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    pick_and_place_msgs__action__ExecuteGraspMove_Feedback * data =
      (pick_and_place_msgs__action__ExecuteGraspMove_Feedback *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!pick_and_place_msgs__action__ExecuteGraspMove_Feedback__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          pick_and_place_msgs__action__ExecuteGraspMove_Feedback__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!pick_and_place_msgs__action__ExecuteGraspMove_Feedback__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}


// Include directives for member types
// Member `goal_id`
#include "unique_identifier_msgs/msg/detail/uuid__functions.h"
// Member `goal`
// already included above
// #include "pick_and_place_msgs/action/detail/execute_grasp_move__functions.h"

bool
pick_and_place_msgs__action__ExecuteGraspMove_SendGoal_Request__init(pick_and_place_msgs__action__ExecuteGraspMove_SendGoal_Request * msg)
{
  if (!msg) {
    return false;
  }
  // goal_id
  if (!unique_identifier_msgs__msg__UUID__init(&msg->goal_id)) {
    pick_and_place_msgs__action__ExecuteGraspMove_SendGoal_Request__fini(msg);
    return false;
  }
  // goal
  if (!pick_and_place_msgs__action__ExecuteGraspMove_Goal__init(&msg->goal)) {
    pick_and_place_msgs__action__ExecuteGraspMove_SendGoal_Request__fini(msg);
    return false;
  }
  return true;
}

void
pick_and_place_msgs__action__ExecuteGraspMove_SendGoal_Request__fini(pick_and_place_msgs__action__ExecuteGraspMove_SendGoal_Request * msg)
{
  if (!msg) {
    return;
  }
  // goal_id
  unique_identifier_msgs__msg__UUID__fini(&msg->goal_id);
  // goal
  pick_and_place_msgs__action__ExecuteGraspMove_Goal__fini(&msg->goal);
}

bool
pick_and_place_msgs__action__ExecuteGraspMove_SendGoal_Request__are_equal(const pick_and_place_msgs__action__ExecuteGraspMove_SendGoal_Request * lhs, const pick_and_place_msgs__action__ExecuteGraspMove_SendGoal_Request * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // goal_id
  if (!unique_identifier_msgs__msg__UUID__are_equal(
      &(lhs->goal_id), &(rhs->goal_id)))
  {
    return false;
  }
  // goal
  if (!pick_and_place_msgs__action__ExecuteGraspMove_Goal__are_equal(
      &(lhs->goal), &(rhs->goal)))
  {
    return false;
  }
  return true;
}

bool
pick_and_place_msgs__action__ExecuteGraspMove_SendGoal_Request__copy(
  const pick_and_place_msgs__action__ExecuteGraspMove_SendGoal_Request * input,
  pick_and_place_msgs__action__ExecuteGraspMove_SendGoal_Request * output)
{
  if (!input || !output) {
    return false;
  }
  // goal_id
  if (!unique_identifier_msgs__msg__UUID__copy(
      &(input->goal_id), &(output->goal_id)))
  {
    return false;
  }
  // goal
  if (!pick_and_place_msgs__action__ExecuteGraspMove_Goal__copy(
      &(input->goal), &(output->goal)))
  {
    return false;
  }
  return true;
}

pick_and_place_msgs__action__ExecuteGraspMove_SendGoal_Request *
pick_and_place_msgs__action__ExecuteGraspMove_SendGoal_Request__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  pick_and_place_msgs__action__ExecuteGraspMove_SendGoal_Request * msg = (pick_and_place_msgs__action__ExecuteGraspMove_SendGoal_Request *)allocator.allocate(sizeof(pick_and_place_msgs__action__ExecuteGraspMove_SendGoal_Request), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(pick_and_place_msgs__action__ExecuteGraspMove_SendGoal_Request));
  bool success = pick_and_place_msgs__action__ExecuteGraspMove_SendGoal_Request__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
pick_and_place_msgs__action__ExecuteGraspMove_SendGoal_Request__destroy(pick_and_place_msgs__action__ExecuteGraspMove_SendGoal_Request * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    pick_and_place_msgs__action__ExecuteGraspMove_SendGoal_Request__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
pick_and_place_msgs__action__ExecuteGraspMove_SendGoal_Request__Sequence__init(pick_and_place_msgs__action__ExecuteGraspMove_SendGoal_Request__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  pick_and_place_msgs__action__ExecuteGraspMove_SendGoal_Request * data = NULL;

  if (size) {
    data = (pick_and_place_msgs__action__ExecuteGraspMove_SendGoal_Request *)allocator.zero_allocate(size, sizeof(pick_and_place_msgs__action__ExecuteGraspMove_SendGoal_Request), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = pick_and_place_msgs__action__ExecuteGraspMove_SendGoal_Request__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        pick_and_place_msgs__action__ExecuteGraspMove_SendGoal_Request__fini(&data[i - 1]);
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
pick_and_place_msgs__action__ExecuteGraspMove_SendGoal_Request__Sequence__fini(pick_and_place_msgs__action__ExecuteGraspMove_SendGoal_Request__Sequence * array)
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
      pick_and_place_msgs__action__ExecuteGraspMove_SendGoal_Request__fini(&array->data[i]);
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

pick_and_place_msgs__action__ExecuteGraspMove_SendGoal_Request__Sequence *
pick_and_place_msgs__action__ExecuteGraspMove_SendGoal_Request__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  pick_and_place_msgs__action__ExecuteGraspMove_SendGoal_Request__Sequence * array = (pick_and_place_msgs__action__ExecuteGraspMove_SendGoal_Request__Sequence *)allocator.allocate(sizeof(pick_and_place_msgs__action__ExecuteGraspMove_SendGoal_Request__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = pick_and_place_msgs__action__ExecuteGraspMove_SendGoal_Request__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
pick_and_place_msgs__action__ExecuteGraspMove_SendGoal_Request__Sequence__destroy(pick_and_place_msgs__action__ExecuteGraspMove_SendGoal_Request__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    pick_and_place_msgs__action__ExecuteGraspMove_SendGoal_Request__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
pick_and_place_msgs__action__ExecuteGraspMove_SendGoal_Request__Sequence__are_equal(const pick_and_place_msgs__action__ExecuteGraspMove_SendGoal_Request__Sequence * lhs, const pick_and_place_msgs__action__ExecuteGraspMove_SendGoal_Request__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!pick_and_place_msgs__action__ExecuteGraspMove_SendGoal_Request__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
pick_and_place_msgs__action__ExecuteGraspMove_SendGoal_Request__Sequence__copy(
  const pick_and_place_msgs__action__ExecuteGraspMove_SendGoal_Request__Sequence * input,
  pick_and_place_msgs__action__ExecuteGraspMove_SendGoal_Request__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(pick_and_place_msgs__action__ExecuteGraspMove_SendGoal_Request);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    pick_and_place_msgs__action__ExecuteGraspMove_SendGoal_Request * data =
      (pick_and_place_msgs__action__ExecuteGraspMove_SendGoal_Request *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!pick_and_place_msgs__action__ExecuteGraspMove_SendGoal_Request__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          pick_and_place_msgs__action__ExecuteGraspMove_SendGoal_Request__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!pick_and_place_msgs__action__ExecuteGraspMove_SendGoal_Request__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}


// Include directives for member types
// Member `stamp`
#include "builtin_interfaces/msg/detail/time__functions.h"

bool
pick_and_place_msgs__action__ExecuteGraspMove_SendGoal_Response__init(pick_and_place_msgs__action__ExecuteGraspMove_SendGoal_Response * msg)
{
  if (!msg) {
    return false;
  }
  // accepted
  // stamp
  if (!builtin_interfaces__msg__Time__init(&msg->stamp)) {
    pick_and_place_msgs__action__ExecuteGraspMove_SendGoal_Response__fini(msg);
    return false;
  }
  return true;
}

void
pick_and_place_msgs__action__ExecuteGraspMove_SendGoal_Response__fini(pick_and_place_msgs__action__ExecuteGraspMove_SendGoal_Response * msg)
{
  if (!msg) {
    return;
  }
  // accepted
  // stamp
  builtin_interfaces__msg__Time__fini(&msg->stamp);
}

bool
pick_and_place_msgs__action__ExecuteGraspMove_SendGoal_Response__are_equal(const pick_and_place_msgs__action__ExecuteGraspMove_SendGoal_Response * lhs, const pick_and_place_msgs__action__ExecuteGraspMove_SendGoal_Response * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // accepted
  if (lhs->accepted != rhs->accepted) {
    return false;
  }
  // stamp
  if (!builtin_interfaces__msg__Time__are_equal(
      &(lhs->stamp), &(rhs->stamp)))
  {
    return false;
  }
  return true;
}

bool
pick_and_place_msgs__action__ExecuteGraspMove_SendGoal_Response__copy(
  const pick_and_place_msgs__action__ExecuteGraspMove_SendGoal_Response * input,
  pick_and_place_msgs__action__ExecuteGraspMove_SendGoal_Response * output)
{
  if (!input || !output) {
    return false;
  }
  // accepted
  output->accepted = input->accepted;
  // stamp
  if (!builtin_interfaces__msg__Time__copy(
      &(input->stamp), &(output->stamp)))
  {
    return false;
  }
  return true;
}

pick_and_place_msgs__action__ExecuteGraspMove_SendGoal_Response *
pick_and_place_msgs__action__ExecuteGraspMove_SendGoal_Response__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  pick_and_place_msgs__action__ExecuteGraspMove_SendGoal_Response * msg = (pick_and_place_msgs__action__ExecuteGraspMove_SendGoal_Response *)allocator.allocate(sizeof(pick_and_place_msgs__action__ExecuteGraspMove_SendGoal_Response), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(pick_and_place_msgs__action__ExecuteGraspMove_SendGoal_Response));
  bool success = pick_and_place_msgs__action__ExecuteGraspMove_SendGoal_Response__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
pick_and_place_msgs__action__ExecuteGraspMove_SendGoal_Response__destroy(pick_and_place_msgs__action__ExecuteGraspMove_SendGoal_Response * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    pick_and_place_msgs__action__ExecuteGraspMove_SendGoal_Response__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
pick_and_place_msgs__action__ExecuteGraspMove_SendGoal_Response__Sequence__init(pick_and_place_msgs__action__ExecuteGraspMove_SendGoal_Response__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  pick_and_place_msgs__action__ExecuteGraspMove_SendGoal_Response * data = NULL;

  if (size) {
    data = (pick_and_place_msgs__action__ExecuteGraspMove_SendGoal_Response *)allocator.zero_allocate(size, sizeof(pick_and_place_msgs__action__ExecuteGraspMove_SendGoal_Response), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = pick_and_place_msgs__action__ExecuteGraspMove_SendGoal_Response__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        pick_and_place_msgs__action__ExecuteGraspMove_SendGoal_Response__fini(&data[i - 1]);
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
pick_and_place_msgs__action__ExecuteGraspMove_SendGoal_Response__Sequence__fini(pick_and_place_msgs__action__ExecuteGraspMove_SendGoal_Response__Sequence * array)
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
      pick_and_place_msgs__action__ExecuteGraspMove_SendGoal_Response__fini(&array->data[i]);
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

pick_and_place_msgs__action__ExecuteGraspMove_SendGoal_Response__Sequence *
pick_and_place_msgs__action__ExecuteGraspMove_SendGoal_Response__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  pick_and_place_msgs__action__ExecuteGraspMove_SendGoal_Response__Sequence * array = (pick_and_place_msgs__action__ExecuteGraspMove_SendGoal_Response__Sequence *)allocator.allocate(sizeof(pick_and_place_msgs__action__ExecuteGraspMove_SendGoal_Response__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = pick_and_place_msgs__action__ExecuteGraspMove_SendGoal_Response__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
pick_and_place_msgs__action__ExecuteGraspMove_SendGoal_Response__Sequence__destroy(pick_and_place_msgs__action__ExecuteGraspMove_SendGoal_Response__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    pick_and_place_msgs__action__ExecuteGraspMove_SendGoal_Response__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
pick_and_place_msgs__action__ExecuteGraspMove_SendGoal_Response__Sequence__are_equal(const pick_and_place_msgs__action__ExecuteGraspMove_SendGoal_Response__Sequence * lhs, const pick_and_place_msgs__action__ExecuteGraspMove_SendGoal_Response__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!pick_and_place_msgs__action__ExecuteGraspMove_SendGoal_Response__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
pick_and_place_msgs__action__ExecuteGraspMove_SendGoal_Response__Sequence__copy(
  const pick_and_place_msgs__action__ExecuteGraspMove_SendGoal_Response__Sequence * input,
  pick_and_place_msgs__action__ExecuteGraspMove_SendGoal_Response__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(pick_and_place_msgs__action__ExecuteGraspMove_SendGoal_Response);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    pick_and_place_msgs__action__ExecuteGraspMove_SendGoal_Response * data =
      (pick_and_place_msgs__action__ExecuteGraspMove_SendGoal_Response *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!pick_and_place_msgs__action__ExecuteGraspMove_SendGoal_Response__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          pick_and_place_msgs__action__ExecuteGraspMove_SendGoal_Response__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!pick_and_place_msgs__action__ExecuteGraspMove_SendGoal_Response__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}


// Include directives for member types
// Member `goal_id`
// already included above
// #include "unique_identifier_msgs/msg/detail/uuid__functions.h"

bool
pick_and_place_msgs__action__ExecuteGraspMove_GetResult_Request__init(pick_and_place_msgs__action__ExecuteGraspMove_GetResult_Request * msg)
{
  if (!msg) {
    return false;
  }
  // goal_id
  if (!unique_identifier_msgs__msg__UUID__init(&msg->goal_id)) {
    pick_and_place_msgs__action__ExecuteGraspMove_GetResult_Request__fini(msg);
    return false;
  }
  return true;
}

void
pick_and_place_msgs__action__ExecuteGraspMove_GetResult_Request__fini(pick_and_place_msgs__action__ExecuteGraspMove_GetResult_Request * msg)
{
  if (!msg) {
    return;
  }
  // goal_id
  unique_identifier_msgs__msg__UUID__fini(&msg->goal_id);
}

bool
pick_and_place_msgs__action__ExecuteGraspMove_GetResult_Request__are_equal(const pick_and_place_msgs__action__ExecuteGraspMove_GetResult_Request * lhs, const pick_and_place_msgs__action__ExecuteGraspMove_GetResult_Request * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // goal_id
  if (!unique_identifier_msgs__msg__UUID__are_equal(
      &(lhs->goal_id), &(rhs->goal_id)))
  {
    return false;
  }
  return true;
}

bool
pick_and_place_msgs__action__ExecuteGraspMove_GetResult_Request__copy(
  const pick_and_place_msgs__action__ExecuteGraspMove_GetResult_Request * input,
  pick_and_place_msgs__action__ExecuteGraspMove_GetResult_Request * output)
{
  if (!input || !output) {
    return false;
  }
  // goal_id
  if (!unique_identifier_msgs__msg__UUID__copy(
      &(input->goal_id), &(output->goal_id)))
  {
    return false;
  }
  return true;
}

pick_and_place_msgs__action__ExecuteGraspMove_GetResult_Request *
pick_and_place_msgs__action__ExecuteGraspMove_GetResult_Request__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  pick_and_place_msgs__action__ExecuteGraspMove_GetResult_Request * msg = (pick_and_place_msgs__action__ExecuteGraspMove_GetResult_Request *)allocator.allocate(sizeof(pick_and_place_msgs__action__ExecuteGraspMove_GetResult_Request), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(pick_and_place_msgs__action__ExecuteGraspMove_GetResult_Request));
  bool success = pick_and_place_msgs__action__ExecuteGraspMove_GetResult_Request__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
pick_and_place_msgs__action__ExecuteGraspMove_GetResult_Request__destroy(pick_and_place_msgs__action__ExecuteGraspMove_GetResult_Request * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    pick_and_place_msgs__action__ExecuteGraspMove_GetResult_Request__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
pick_and_place_msgs__action__ExecuteGraspMove_GetResult_Request__Sequence__init(pick_and_place_msgs__action__ExecuteGraspMove_GetResult_Request__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  pick_and_place_msgs__action__ExecuteGraspMove_GetResult_Request * data = NULL;

  if (size) {
    data = (pick_and_place_msgs__action__ExecuteGraspMove_GetResult_Request *)allocator.zero_allocate(size, sizeof(pick_and_place_msgs__action__ExecuteGraspMove_GetResult_Request), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = pick_and_place_msgs__action__ExecuteGraspMove_GetResult_Request__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        pick_and_place_msgs__action__ExecuteGraspMove_GetResult_Request__fini(&data[i - 1]);
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
pick_and_place_msgs__action__ExecuteGraspMove_GetResult_Request__Sequence__fini(pick_and_place_msgs__action__ExecuteGraspMove_GetResult_Request__Sequence * array)
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
      pick_and_place_msgs__action__ExecuteGraspMove_GetResult_Request__fini(&array->data[i]);
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

pick_and_place_msgs__action__ExecuteGraspMove_GetResult_Request__Sequence *
pick_and_place_msgs__action__ExecuteGraspMove_GetResult_Request__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  pick_and_place_msgs__action__ExecuteGraspMove_GetResult_Request__Sequence * array = (pick_and_place_msgs__action__ExecuteGraspMove_GetResult_Request__Sequence *)allocator.allocate(sizeof(pick_and_place_msgs__action__ExecuteGraspMove_GetResult_Request__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = pick_and_place_msgs__action__ExecuteGraspMove_GetResult_Request__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
pick_and_place_msgs__action__ExecuteGraspMove_GetResult_Request__Sequence__destroy(pick_and_place_msgs__action__ExecuteGraspMove_GetResult_Request__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    pick_and_place_msgs__action__ExecuteGraspMove_GetResult_Request__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
pick_and_place_msgs__action__ExecuteGraspMove_GetResult_Request__Sequence__are_equal(const pick_and_place_msgs__action__ExecuteGraspMove_GetResult_Request__Sequence * lhs, const pick_and_place_msgs__action__ExecuteGraspMove_GetResult_Request__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!pick_and_place_msgs__action__ExecuteGraspMove_GetResult_Request__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
pick_and_place_msgs__action__ExecuteGraspMove_GetResult_Request__Sequence__copy(
  const pick_and_place_msgs__action__ExecuteGraspMove_GetResult_Request__Sequence * input,
  pick_and_place_msgs__action__ExecuteGraspMove_GetResult_Request__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(pick_and_place_msgs__action__ExecuteGraspMove_GetResult_Request);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    pick_and_place_msgs__action__ExecuteGraspMove_GetResult_Request * data =
      (pick_and_place_msgs__action__ExecuteGraspMove_GetResult_Request *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!pick_and_place_msgs__action__ExecuteGraspMove_GetResult_Request__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          pick_and_place_msgs__action__ExecuteGraspMove_GetResult_Request__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!pick_and_place_msgs__action__ExecuteGraspMove_GetResult_Request__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}


// Include directives for member types
// Member `result`
// already included above
// #include "pick_and_place_msgs/action/detail/execute_grasp_move__functions.h"

bool
pick_and_place_msgs__action__ExecuteGraspMove_GetResult_Response__init(pick_and_place_msgs__action__ExecuteGraspMove_GetResult_Response * msg)
{
  if (!msg) {
    return false;
  }
  // status
  // result
  if (!pick_and_place_msgs__action__ExecuteGraspMove_Result__init(&msg->result)) {
    pick_and_place_msgs__action__ExecuteGraspMove_GetResult_Response__fini(msg);
    return false;
  }
  return true;
}

void
pick_and_place_msgs__action__ExecuteGraspMove_GetResult_Response__fini(pick_and_place_msgs__action__ExecuteGraspMove_GetResult_Response * msg)
{
  if (!msg) {
    return;
  }
  // status
  // result
  pick_and_place_msgs__action__ExecuteGraspMove_Result__fini(&msg->result);
}

bool
pick_and_place_msgs__action__ExecuteGraspMove_GetResult_Response__are_equal(const pick_and_place_msgs__action__ExecuteGraspMove_GetResult_Response * lhs, const pick_and_place_msgs__action__ExecuteGraspMove_GetResult_Response * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // status
  if (lhs->status != rhs->status) {
    return false;
  }
  // result
  if (!pick_and_place_msgs__action__ExecuteGraspMove_Result__are_equal(
      &(lhs->result), &(rhs->result)))
  {
    return false;
  }
  return true;
}

bool
pick_and_place_msgs__action__ExecuteGraspMove_GetResult_Response__copy(
  const pick_and_place_msgs__action__ExecuteGraspMove_GetResult_Response * input,
  pick_and_place_msgs__action__ExecuteGraspMove_GetResult_Response * output)
{
  if (!input || !output) {
    return false;
  }
  // status
  output->status = input->status;
  // result
  if (!pick_and_place_msgs__action__ExecuteGraspMove_Result__copy(
      &(input->result), &(output->result)))
  {
    return false;
  }
  return true;
}

pick_and_place_msgs__action__ExecuteGraspMove_GetResult_Response *
pick_and_place_msgs__action__ExecuteGraspMove_GetResult_Response__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  pick_and_place_msgs__action__ExecuteGraspMove_GetResult_Response * msg = (pick_and_place_msgs__action__ExecuteGraspMove_GetResult_Response *)allocator.allocate(sizeof(pick_and_place_msgs__action__ExecuteGraspMove_GetResult_Response), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(pick_and_place_msgs__action__ExecuteGraspMove_GetResult_Response));
  bool success = pick_and_place_msgs__action__ExecuteGraspMove_GetResult_Response__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
pick_and_place_msgs__action__ExecuteGraspMove_GetResult_Response__destroy(pick_and_place_msgs__action__ExecuteGraspMove_GetResult_Response * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    pick_and_place_msgs__action__ExecuteGraspMove_GetResult_Response__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
pick_and_place_msgs__action__ExecuteGraspMove_GetResult_Response__Sequence__init(pick_and_place_msgs__action__ExecuteGraspMove_GetResult_Response__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  pick_and_place_msgs__action__ExecuteGraspMove_GetResult_Response * data = NULL;

  if (size) {
    data = (pick_and_place_msgs__action__ExecuteGraspMove_GetResult_Response *)allocator.zero_allocate(size, sizeof(pick_and_place_msgs__action__ExecuteGraspMove_GetResult_Response), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = pick_and_place_msgs__action__ExecuteGraspMove_GetResult_Response__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        pick_and_place_msgs__action__ExecuteGraspMove_GetResult_Response__fini(&data[i - 1]);
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
pick_and_place_msgs__action__ExecuteGraspMove_GetResult_Response__Sequence__fini(pick_and_place_msgs__action__ExecuteGraspMove_GetResult_Response__Sequence * array)
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
      pick_and_place_msgs__action__ExecuteGraspMove_GetResult_Response__fini(&array->data[i]);
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

pick_and_place_msgs__action__ExecuteGraspMove_GetResult_Response__Sequence *
pick_and_place_msgs__action__ExecuteGraspMove_GetResult_Response__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  pick_and_place_msgs__action__ExecuteGraspMove_GetResult_Response__Sequence * array = (pick_and_place_msgs__action__ExecuteGraspMove_GetResult_Response__Sequence *)allocator.allocate(sizeof(pick_and_place_msgs__action__ExecuteGraspMove_GetResult_Response__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = pick_and_place_msgs__action__ExecuteGraspMove_GetResult_Response__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
pick_and_place_msgs__action__ExecuteGraspMove_GetResult_Response__Sequence__destroy(pick_and_place_msgs__action__ExecuteGraspMove_GetResult_Response__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    pick_and_place_msgs__action__ExecuteGraspMove_GetResult_Response__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
pick_and_place_msgs__action__ExecuteGraspMove_GetResult_Response__Sequence__are_equal(const pick_and_place_msgs__action__ExecuteGraspMove_GetResult_Response__Sequence * lhs, const pick_and_place_msgs__action__ExecuteGraspMove_GetResult_Response__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!pick_and_place_msgs__action__ExecuteGraspMove_GetResult_Response__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
pick_and_place_msgs__action__ExecuteGraspMove_GetResult_Response__Sequence__copy(
  const pick_and_place_msgs__action__ExecuteGraspMove_GetResult_Response__Sequence * input,
  pick_and_place_msgs__action__ExecuteGraspMove_GetResult_Response__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(pick_and_place_msgs__action__ExecuteGraspMove_GetResult_Response);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    pick_and_place_msgs__action__ExecuteGraspMove_GetResult_Response * data =
      (pick_and_place_msgs__action__ExecuteGraspMove_GetResult_Response *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!pick_and_place_msgs__action__ExecuteGraspMove_GetResult_Response__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          pick_and_place_msgs__action__ExecuteGraspMove_GetResult_Response__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!pick_and_place_msgs__action__ExecuteGraspMove_GetResult_Response__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}


// Include directives for member types
// Member `goal_id`
// already included above
// #include "unique_identifier_msgs/msg/detail/uuid__functions.h"
// Member `feedback`
// already included above
// #include "pick_and_place_msgs/action/detail/execute_grasp_move__functions.h"

bool
pick_and_place_msgs__action__ExecuteGraspMove_FeedbackMessage__init(pick_and_place_msgs__action__ExecuteGraspMove_FeedbackMessage * msg)
{
  if (!msg) {
    return false;
  }
  // goal_id
  if (!unique_identifier_msgs__msg__UUID__init(&msg->goal_id)) {
    pick_and_place_msgs__action__ExecuteGraspMove_FeedbackMessage__fini(msg);
    return false;
  }
  // feedback
  if (!pick_and_place_msgs__action__ExecuteGraspMove_Feedback__init(&msg->feedback)) {
    pick_and_place_msgs__action__ExecuteGraspMove_FeedbackMessage__fini(msg);
    return false;
  }
  return true;
}

void
pick_and_place_msgs__action__ExecuteGraspMove_FeedbackMessage__fini(pick_and_place_msgs__action__ExecuteGraspMove_FeedbackMessage * msg)
{
  if (!msg) {
    return;
  }
  // goal_id
  unique_identifier_msgs__msg__UUID__fini(&msg->goal_id);
  // feedback
  pick_and_place_msgs__action__ExecuteGraspMove_Feedback__fini(&msg->feedback);
}

bool
pick_and_place_msgs__action__ExecuteGraspMove_FeedbackMessage__are_equal(const pick_and_place_msgs__action__ExecuteGraspMove_FeedbackMessage * lhs, const pick_and_place_msgs__action__ExecuteGraspMove_FeedbackMessage * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // goal_id
  if (!unique_identifier_msgs__msg__UUID__are_equal(
      &(lhs->goal_id), &(rhs->goal_id)))
  {
    return false;
  }
  // feedback
  if (!pick_and_place_msgs__action__ExecuteGraspMove_Feedback__are_equal(
      &(lhs->feedback), &(rhs->feedback)))
  {
    return false;
  }
  return true;
}

bool
pick_and_place_msgs__action__ExecuteGraspMove_FeedbackMessage__copy(
  const pick_and_place_msgs__action__ExecuteGraspMove_FeedbackMessage * input,
  pick_and_place_msgs__action__ExecuteGraspMove_FeedbackMessage * output)
{
  if (!input || !output) {
    return false;
  }
  // goal_id
  if (!unique_identifier_msgs__msg__UUID__copy(
      &(input->goal_id), &(output->goal_id)))
  {
    return false;
  }
  // feedback
  if (!pick_and_place_msgs__action__ExecuteGraspMove_Feedback__copy(
      &(input->feedback), &(output->feedback)))
  {
    return false;
  }
  return true;
}

pick_and_place_msgs__action__ExecuteGraspMove_FeedbackMessage *
pick_and_place_msgs__action__ExecuteGraspMove_FeedbackMessage__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  pick_and_place_msgs__action__ExecuteGraspMove_FeedbackMessage * msg = (pick_and_place_msgs__action__ExecuteGraspMove_FeedbackMessage *)allocator.allocate(sizeof(pick_and_place_msgs__action__ExecuteGraspMove_FeedbackMessage), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(pick_and_place_msgs__action__ExecuteGraspMove_FeedbackMessage));
  bool success = pick_and_place_msgs__action__ExecuteGraspMove_FeedbackMessage__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
pick_and_place_msgs__action__ExecuteGraspMove_FeedbackMessage__destroy(pick_and_place_msgs__action__ExecuteGraspMove_FeedbackMessage * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    pick_and_place_msgs__action__ExecuteGraspMove_FeedbackMessage__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
pick_and_place_msgs__action__ExecuteGraspMove_FeedbackMessage__Sequence__init(pick_and_place_msgs__action__ExecuteGraspMove_FeedbackMessage__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  pick_and_place_msgs__action__ExecuteGraspMove_FeedbackMessage * data = NULL;

  if (size) {
    data = (pick_and_place_msgs__action__ExecuteGraspMove_FeedbackMessage *)allocator.zero_allocate(size, sizeof(pick_and_place_msgs__action__ExecuteGraspMove_FeedbackMessage), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = pick_and_place_msgs__action__ExecuteGraspMove_FeedbackMessage__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        pick_and_place_msgs__action__ExecuteGraspMove_FeedbackMessage__fini(&data[i - 1]);
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
pick_and_place_msgs__action__ExecuteGraspMove_FeedbackMessage__Sequence__fini(pick_and_place_msgs__action__ExecuteGraspMove_FeedbackMessage__Sequence * array)
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
      pick_and_place_msgs__action__ExecuteGraspMove_FeedbackMessage__fini(&array->data[i]);
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

pick_and_place_msgs__action__ExecuteGraspMove_FeedbackMessage__Sequence *
pick_and_place_msgs__action__ExecuteGraspMove_FeedbackMessage__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  pick_and_place_msgs__action__ExecuteGraspMove_FeedbackMessage__Sequence * array = (pick_and_place_msgs__action__ExecuteGraspMove_FeedbackMessage__Sequence *)allocator.allocate(sizeof(pick_and_place_msgs__action__ExecuteGraspMove_FeedbackMessage__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = pick_and_place_msgs__action__ExecuteGraspMove_FeedbackMessage__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
pick_and_place_msgs__action__ExecuteGraspMove_FeedbackMessage__Sequence__destroy(pick_and_place_msgs__action__ExecuteGraspMove_FeedbackMessage__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    pick_and_place_msgs__action__ExecuteGraspMove_FeedbackMessage__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
pick_and_place_msgs__action__ExecuteGraspMove_FeedbackMessage__Sequence__are_equal(const pick_and_place_msgs__action__ExecuteGraspMove_FeedbackMessage__Sequence * lhs, const pick_and_place_msgs__action__ExecuteGraspMove_FeedbackMessage__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!pick_and_place_msgs__action__ExecuteGraspMove_FeedbackMessage__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
pick_and_place_msgs__action__ExecuteGraspMove_FeedbackMessage__Sequence__copy(
  const pick_and_place_msgs__action__ExecuteGraspMove_FeedbackMessage__Sequence * input,
  pick_and_place_msgs__action__ExecuteGraspMove_FeedbackMessage__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(pick_and_place_msgs__action__ExecuteGraspMove_FeedbackMessage);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    pick_and_place_msgs__action__ExecuteGraspMove_FeedbackMessage * data =
      (pick_and_place_msgs__action__ExecuteGraspMove_FeedbackMessage *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!pick_and_place_msgs__action__ExecuteGraspMove_FeedbackMessage__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          pick_and_place_msgs__action__ExecuteGraspMove_FeedbackMessage__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!pick_and_place_msgs__action__ExecuteGraspMove_FeedbackMessage__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
