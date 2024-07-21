// generated from rosidl_generator_c/resource/idl__functions.h.em
// with input from pick_and_place_msgs:srv/GetTargetPose.idl
// generated code does not contain a copyright notice

#ifndef PICK_AND_PLACE_MSGS__SRV__DETAIL__GET_TARGET_POSE__FUNCTIONS_H_
#define PICK_AND_PLACE_MSGS__SRV__DETAIL__GET_TARGET_POSE__FUNCTIONS_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stdlib.h>

#include "rosidl_runtime_c/visibility_control.h"
#include "pick_and_place_msgs/msg/rosidl_generator_c__visibility_control.h"

#include "pick_and_place_msgs/srv/detail/get_target_pose__struct.h"

/// Initialize srv/GetTargetPose message.
/**
 * If the init function is called twice for the same message without
 * calling fini inbetween previously allocated memory will be leaked.
 * \param[in,out] msg The previously allocated message pointer.
 * Fields without a default value will not be initialized by this function.
 * You might want to call memset(msg, 0, sizeof(
 * pick_and_place_msgs__srv__GetTargetPose_Request
 * )) before or use
 * pick_and_place_msgs__srv__GetTargetPose_Request__create()
 * to allocate and initialize the message.
 * \return true if initialization was successful, otherwise false
 */
ROSIDL_GENERATOR_C_PUBLIC_pick_and_place_msgs
bool
pick_and_place_msgs__srv__GetTargetPose_Request__init(pick_and_place_msgs__srv__GetTargetPose_Request * msg);

/// Finalize srv/GetTargetPose message.
/**
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_pick_and_place_msgs
void
pick_and_place_msgs__srv__GetTargetPose_Request__fini(pick_and_place_msgs__srv__GetTargetPose_Request * msg);

/// Create srv/GetTargetPose message.
/**
 * It allocates the memory for the message, sets the memory to zero, and
 * calls
 * pick_and_place_msgs__srv__GetTargetPose_Request__init().
 * \return The pointer to the initialized message if successful,
 * otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_pick_and_place_msgs
pick_and_place_msgs__srv__GetTargetPose_Request *
pick_and_place_msgs__srv__GetTargetPose_Request__create();

/// Destroy srv/GetTargetPose message.
/**
 * It calls
 * pick_and_place_msgs__srv__GetTargetPose_Request__fini()
 * and frees the memory of the message.
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_pick_and_place_msgs
void
pick_and_place_msgs__srv__GetTargetPose_Request__destroy(pick_and_place_msgs__srv__GetTargetPose_Request * msg);

/// Check for srv/GetTargetPose message equality.
/**
 * \param[in] lhs The message on the left hand size of the equality operator.
 * \param[in] rhs The message on the right hand size of the equality operator.
 * \return true if messages are equal, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_pick_and_place_msgs
bool
pick_and_place_msgs__srv__GetTargetPose_Request__are_equal(const pick_and_place_msgs__srv__GetTargetPose_Request * lhs, const pick_and_place_msgs__srv__GetTargetPose_Request * rhs);

/// Copy a srv/GetTargetPose message.
/**
 * This functions performs a deep copy, as opposed to the shallow copy that
 * plain assignment yields.
 *
 * \param[in] input The source message pointer.
 * \param[out] output The target message pointer, which must
 *   have been initialized before calling this function.
 * \return true if successful, or false if either pointer is null
 *   or memory allocation fails.
 */
ROSIDL_GENERATOR_C_PUBLIC_pick_and_place_msgs
bool
pick_and_place_msgs__srv__GetTargetPose_Request__copy(
  const pick_and_place_msgs__srv__GetTargetPose_Request * input,
  pick_and_place_msgs__srv__GetTargetPose_Request * output);

/// Initialize array of srv/GetTargetPose messages.
/**
 * It allocates the memory for the number of elements and calls
 * pick_and_place_msgs__srv__GetTargetPose_Request__init()
 * for each element of the array.
 * \param[in,out] array The allocated array pointer.
 * \param[in] size The size / capacity of the array.
 * \return true if initialization was successful, otherwise false
 * If the array pointer is valid and the size is zero it is guaranteed
 # to return true.
 */
ROSIDL_GENERATOR_C_PUBLIC_pick_and_place_msgs
bool
pick_and_place_msgs__srv__GetTargetPose_Request__Sequence__init(pick_and_place_msgs__srv__GetTargetPose_Request__Sequence * array, size_t size);

/// Finalize array of srv/GetTargetPose messages.
/**
 * It calls
 * pick_and_place_msgs__srv__GetTargetPose_Request__fini()
 * for each element of the array and frees the memory for the number of
 * elements.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_pick_and_place_msgs
void
pick_and_place_msgs__srv__GetTargetPose_Request__Sequence__fini(pick_and_place_msgs__srv__GetTargetPose_Request__Sequence * array);

/// Create array of srv/GetTargetPose messages.
/**
 * It allocates the memory for the array and calls
 * pick_and_place_msgs__srv__GetTargetPose_Request__Sequence__init().
 * \param[in] size The size / capacity of the array.
 * \return The pointer to the initialized array if successful, otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_pick_and_place_msgs
pick_and_place_msgs__srv__GetTargetPose_Request__Sequence *
pick_and_place_msgs__srv__GetTargetPose_Request__Sequence__create(size_t size);

/// Destroy array of srv/GetTargetPose messages.
/**
 * It calls
 * pick_and_place_msgs__srv__GetTargetPose_Request__Sequence__fini()
 * on the array,
 * and frees the memory of the array.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_pick_and_place_msgs
void
pick_and_place_msgs__srv__GetTargetPose_Request__Sequence__destroy(pick_and_place_msgs__srv__GetTargetPose_Request__Sequence * array);

/// Check for srv/GetTargetPose message array equality.
/**
 * \param[in] lhs The message array on the left hand size of the equality operator.
 * \param[in] rhs The message array on the right hand size of the equality operator.
 * \return true if message arrays are equal in size and content, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_pick_and_place_msgs
bool
pick_and_place_msgs__srv__GetTargetPose_Request__Sequence__are_equal(const pick_and_place_msgs__srv__GetTargetPose_Request__Sequence * lhs, const pick_and_place_msgs__srv__GetTargetPose_Request__Sequence * rhs);

/// Copy an array of srv/GetTargetPose messages.
/**
 * This functions performs a deep copy, as opposed to the shallow copy that
 * plain assignment yields.
 *
 * \param[in] input The source array pointer.
 * \param[out] output The target array pointer, which must
 *   have been initialized before calling this function.
 * \return true if successful, or false if either pointer
 *   is null or memory allocation fails.
 */
ROSIDL_GENERATOR_C_PUBLIC_pick_and_place_msgs
bool
pick_and_place_msgs__srv__GetTargetPose_Request__Sequence__copy(
  const pick_and_place_msgs__srv__GetTargetPose_Request__Sequence * input,
  pick_and_place_msgs__srv__GetTargetPose_Request__Sequence * output);

/// Initialize srv/GetTargetPose message.
/**
 * If the init function is called twice for the same message without
 * calling fini inbetween previously allocated memory will be leaked.
 * \param[in,out] msg The previously allocated message pointer.
 * Fields without a default value will not be initialized by this function.
 * You might want to call memset(msg, 0, sizeof(
 * pick_and_place_msgs__srv__GetTargetPose_Response
 * )) before or use
 * pick_and_place_msgs__srv__GetTargetPose_Response__create()
 * to allocate and initialize the message.
 * \return true if initialization was successful, otherwise false
 */
ROSIDL_GENERATOR_C_PUBLIC_pick_and_place_msgs
bool
pick_and_place_msgs__srv__GetTargetPose_Response__init(pick_and_place_msgs__srv__GetTargetPose_Response * msg);

/// Finalize srv/GetTargetPose message.
/**
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_pick_and_place_msgs
void
pick_and_place_msgs__srv__GetTargetPose_Response__fini(pick_and_place_msgs__srv__GetTargetPose_Response * msg);

/// Create srv/GetTargetPose message.
/**
 * It allocates the memory for the message, sets the memory to zero, and
 * calls
 * pick_and_place_msgs__srv__GetTargetPose_Response__init().
 * \return The pointer to the initialized message if successful,
 * otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_pick_and_place_msgs
pick_and_place_msgs__srv__GetTargetPose_Response *
pick_and_place_msgs__srv__GetTargetPose_Response__create();

/// Destroy srv/GetTargetPose message.
/**
 * It calls
 * pick_and_place_msgs__srv__GetTargetPose_Response__fini()
 * and frees the memory of the message.
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_pick_and_place_msgs
void
pick_and_place_msgs__srv__GetTargetPose_Response__destroy(pick_and_place_msgs__srv__GetTargetPose_Response * msg);

/// Check for srv/GetTargetPose message equality.
/**
 * \param[in] lhs The message on the left hand size of the equality operator.
 * \param[in] rhs The message on the right hand size of the equality operator.
 * \return true if messages are equal, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_pick_and_place_msgs
bool
pick_and_place_msgs__srv__GetTargetPose_Response__are_equal(const pick_and_place_msgs__srv__GetTargetPose_Response * lhs, const pick_and_place_msgs__srv__GetTargetPose_Response * rhs);

/// Copy a srv/GetTargetPose message.
/**
 * This functions performs a deep copy, as opposed to the shallow copy that
 * plain assignment yields.
 *
 * \param[in] input The source message pointer.
 * \param[out] output The target message pointer, which must
 *   have been initialized before calling this function.
 * \return true if successful, or false if either pointer is null
 *   or memory allocation fails.
 */
ROSIDL_GENERATOR_C_PUBLIC_pick_and_place_msgs
bool
pick_and_place_msgs__srv__GetTargetPose_Response__copy(
  const pick_and_place_msgs__srv__GetTargetPose_Response * input,
  pick_and_place_msgs__srv__GetTargetPose_Response * output);

/// Initialize array of srv/GetTargetPose messages.
/**
 * It allocates the memory for the number of elements and calls
 * pick_and_place_msgs__srv__GetTargetPose_Response__init()
 * for each element of the array.
 * \param[in,out] array The allocated array pointer.
 * \param[in] size The size / capacity of the array.
 * \return true if initialization was successful, otherwise false
 * If the array pointer is valid and the size is zero it is guaranteed
 # to return true.
 */
ROSIDL_GENERATOR_C_PUBLIC_pick_and_place_msgs
bool
pick_and_place_msgs__srv__GetTargetPose_Response__Sequence__init(pick_and_place_msgs__srv__GetTargetPose_Response__Sequence * array, size_t size);

/// Finalize array of srv/GetTargetPose messages.
/**
 * It calls
 * pick_and_place_msgs__srv__GetTargetPose_Response__fini()
 * for each element of the array and frees the memory for the number of
 * elements.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_pick_and_place_msgs
void
pick_and_place_msgs__srv__GetTargetPose_Response__Sequence__fini(pick_and_place_msgs__srv__GetTargetPose_Response__Sequence * array);

/// Create array of srv/GetTargetPose messages.
/**
 * It allocates the memory for the array and calls
 * pick_and_place_msgs__srv__GetTargetPose_Response__Sequence__init().
 * \param[in] size The size / capacity of the array.
 * \return The pointer to the initialized array if successful, otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_pick_and_place_msgs
pick_and_place_msgs__srv__GetTargetPose_Response__Sequence *
pick_and_place_msgs__srv__GetTargetPose_Response__Sequence__create(size_t size);

/// Destroy array of srv/GetTargetPose messages.
/**
 * It calls
 * pick_and_place_msgs__srv__GetTargetPose_Response__Sequence__fini()
 * on the array,
 * and frees the memory of the array.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_pick_and_place_msgs
void
pick_and_place_msgs__srv__GetTargetPose_Response__Sequence__destroy(pick_and_place_msgs__srv__GetTargetPose_Response__Sequence * array);

/// Check for srv/GetTargetPose message array equality.
/**
 * \param[in] lhs The message array on the left hand size of the equality operator.
 * \param[in] rhs The message array on the right hand size of the equality operator.
 * \return true if message arrays are equal in size and content, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_pick_and_place_msgs
bool
pick_and_place_msgs__srv__GetTargetPose_Response__Sequence__are_equal(const pick_and_place_msgs__srv__GetTargetPose_Response__Sequence * lhs, const pick_and_place_msgs__srv__GetTargetPose_Response__Sequence * rhs);

/// Copy an array of srv/GetTargetPose messages.
/**
 * This functions performs a deep copy, as opposed to the shallow copy that
 * plain assignment yields.
 *
 * \param[in] input The source array pointer.
 * \param[out] output The target array pointer, which must
 *   have been initialized before calling this function.
 * \return true if successful, or false if either pointer
 *   is null or memory allocation fails.
 */
ROSIDL_GENERATOR_C_PUBLIC_pick_and_place_msgs
bool
pick_and_place_msgs__srv__GetTargetPose_Response__Sequence__copy(
  const pick_and_place_msgs__srv__GetTargetPose_Response__Sequence * input,
  pick_and_place_msgs__srv__GetTargetPose_Response__Sequence * output);

#ifdef __cplusplus
}
#endif

#endif  // PICK_AND_PLACE_MSGS__SRV__DETAIL__GET_TARGET_POSE__FUNCTIONS_H_
