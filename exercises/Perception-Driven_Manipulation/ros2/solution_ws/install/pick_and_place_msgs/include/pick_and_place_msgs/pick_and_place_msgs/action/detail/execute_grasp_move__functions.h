// generated from rosidl_generator_c/resource/idl__functions.h.em
// with input from pick_and_place_msgs:action/ExecuteGraspMove.idl
// generated code does not contain a copyright notice

#ifndef PICK_AND_PLACE_MSGS__ACTION__DETAIL__EXECUTE_GRASP_MOVE__FUNCTIONS_H_
#define PICK_AND_PLACE_MSGS__ACTION__DETAIL__EXECUTE_GRASP_MOVE__FUNCTIONS_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stdlib.h>

#include "rosidl_runtime_c/visibility_control.h"
#include "pick_and_place_msgs/msg/rosidl_generator_c__visibility_control.h"

#include "pick_and_place_msgs/action/detail/execute_grasp_move__struct.h"

/// Initialize action/ExecuteGraspMove message.
/**
 * If the init function is called twice for the same message without
 * calling fini inbetween previously allocated memory will be leaked.
 * \param[in,out] msg The previously allocated message pointer.
 * Fields without a default value will not be initialized by this function.
 * You might want to call memset(msg, 0, sizeof(
 * pick_and_place_msgs__action__ExecuteGraspMove_Goal
 * )) before or use
 * pick_and_place_msgs__action__ExecuteGraspMove_Goal__create()
 * to allocate and initialize the message.
 * \return true if initialization was successful, otherwise false
 */
ROSIDL_GENERATOR_C_PUBLIC_pick_and_place_msgs
bool
pick_and_place_msgs__action__ExecuteGraspMove_Goal__init(pick_and_place_msgs__action__ExecuteGraspMove_Goal * msg);

/// Finalize action/ExecuteGraspMove message.
/**
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_pick_and_place_msgs
void
pick_and_place_msgs__action__ExecuteGraspMove_Goal__fini(pick_and_place_msgs__action__ExecuteGraspMove_Goal * msg);

/// Create action/ExecuteGraspMove message.
/**
 * It allocates the memory for the message, sets the memory to zero, and
 * calls
 * pick_and_place_msgs__action__ExecuteGraspMove_Goal__init().
 * \return The pointer to the initialized message if successful,
 * otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_pick_and_place_msgs
pick_and_place_msgs__action__ExecuteGraspMove_Goal *
pick_and_place_msgs__action__ExecuteGraspMove_Goal__create();

/// Destroy action/ExecuteGraspMove message.
/**
 * It calls
 * pick_and_place_msgs__action__ExecuteGraspMove_Goal__fini()
 * and frees the memory of the message.
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_pick_and_place_msgs
void
pick_and_place_msgs__action__ExecuteGraspMove_Goal__destroy(pick_and_place_msgs__action__ExecuteGraspMove_Goal * msg);

/// Check for action/ExecuteGraspMove message equality.
/**
 * \param[in] lhs The message on the left hand size of the equality operator.
 * \param[in] rhs The message on the right hand size of the equality operator.
 * \return true if messages are equal, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_pick_and_place_msgs
bool
pick_and_place_msgs__action__ExecuteGraspMove_Goal__are_equal(const pick_and_place_msgs__action__ExecuteGraspMove_Goal * lhs, const pick_and_place_msgs__action__ExecuteGraspMove_Goal * rhs);

/// Copy a action/ExecuteGraspMove message.
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
pick_and_place_msgs__action__ExecuteGraspMove_Goal__copy(
  const pick_and_place_msgs__action__ExecuteGraspMove_Goal * input,
  pick_and_place_msgs__action__ExecuteGraspMove_Goal * output);

/// Initialize array of action/ExecuteGraspMove messages.
/**
 * It allocates the memory for the number of elements and calls
 * pick_and_place_msgs__action__ExecuteGraspMove_Goal__init()
 * for each element of the array.
 * \param[in,out] array The allocated array pointer.
 * \param[in] size The size / capacity of the array.
 * \return true if initialization was successful, otherwise false
 * If the array pointer is valid and the size is zero it is guaranteed
 # to return true.
 */
ROSIDL_GENERATOR_C_PUBLIC_pick_and_place_msgs
bool
pick_and_place_msgs__action__ExecuteGraspMove_Goal__Sequence__init(pick_and_place_msgs__action__ExecuteGraspMove_Goal__Sequence * array, size_t size);

/// Finalize array of action/ExecuteGraspMove messages.
/**
 * It calls
 * pick_and_place_msgs__action__ExecuteGraspMove_Goal__fini()
 * for each element of the array and frees the memory for the number of
 * elements.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_pick_and_place_msgs
void
pick_and_place_msgs__action__ExecuteGraspMove_Goal__Sequence__fini(pick_and_place_msgs__action__ExecuteGraspMove_Goal__Sequence * array);

/// Create array of action/ExecuteGraspMove messages.
/**
 * It allocates the memory for the array and calls
 * pick_and_place_msgs__action__ExecuteGraspMove_Goal__Sequence__init().
 * \param[in] size The size / capacity of the array.
 * \return The pointer to the initialized array if successful, otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_pick_and_place_msgs
pick_and_place_msgs__action__ExecuteGraspMove_Goal__Sequence *
pick_and_place_msgs__action__ExecuteGraspMove_Goal__Sequence__create(size_t size);

/// Destroy array of action/ExecuteGraspMove messages.
/**
 * It calls
 * pick_and_place_msgs__action__ExecuteGraspMove_Goal__Sequence__fini()
 * on the array,
 * and frees the memory of the array.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_pick_and_place_msgs
void
pick_and_place_msgs__action__ExecuteGraspMove_Goal__Sequence__destroy(pick_and_place_msgs__action__ExecuteGraspMove_Goal__Sequence * array);

/// Check for action/ExecuteGraspMove message array equality.
/**
 * \param[in] lhs The message array on the left hand size of the equality operator.
 * \param[in] rhs The message array on the right hand size of the equality operator.
 * \return true if message arrays are equal in size and content, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_pick_and_place_msgs
bool
pick_and_place_msgs__action__ExecuteGraspMove_Goal__Sequence__are_equal(const pick_and_place_msgs__action__ExecuteGraspMove_Goal__Sequence * lhs, const pick_and_place_msgs__action__ExecuteGraspMove_Goal__Sequence * rhs);

/// Copy an array of action/ExecuteGraspMove messages.
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
pick_and_place_msgs__action__ExecuteGraspMove_Goal__Sequence__copy(
  const pick_and_place_msgs__action__ExecuteGraspMove_Goal__Sequence * input,
  pick_and_place_msgs__action__ExecuteGraspMove_Goal__Sequence * output);

/// Initialize action/ExecuteGraspMove message.
/**
 * If the init function is called twice for the same message without
 * calling fini inbetween previously allocated memory will be leaked.
 * \param[in,out] msg The previously allocated message pointer.
 * Fields without a default value will not be initialized by this function.
 * You might want to call memset(msg, 0, sizeof(
 * pick_and_place_msgs__action__ExecuteGraspMove_Result
 * )) before or use
 * pick_and_place_msgs__action__ExecuteGraspMove_Result__create()
 * to allocate and initialize the message.
 * \return true if initialization was successful, otherwise false
 */
ROSIDL_GENERATOR_C_PUBLIC_pick_and_place_msgs
bool
pick_and_place_msgs__action__ExecuteGraspMove_Result__init(pick_and_place_msgs__action__ExecuteGraspMove_Result * msg);

/// Finalize action/ExecuteGraspMove message.
/**
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_pick_and_place_msgs
void
pick_and_place_msgs__action__ExecuteGraspMove_Result__fini(pick_and_place_msgs__action__ExecuteGraspMove_Result * msg);

/// Create action/ExecuteGraspMove message.
/**
 * It allocates the memory for the message, sets the memory to zero, and
 * calls
 * pick_and_place_msgs__action__ExecuteGraspMove_Result__init().
 * \return The pointer to the initialized message if successful,
 * otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_pick_and_place_msgs
pick_and_place_msgs__action__ExecuteGraspMove_Result *
pick_and_place_msgs__action__ExecuteGraspMove_Result__create();

/// Destroy action/ExecuteGraspMove message.
/**
 * It calls
 * pick_and_place_msgs__action__ExecuteGraspMove_Result__fini()
 * and frees the memory of the message.
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_pick_and_place_msgs
void
pick_and_place_msgs__action__ExecuteGraspMove_Result__destroy(pick_and_place_msgs__action__ExecuteGraspMove_Result * msg);

/// Check for action/ExecuteGraspMove message equality.
/**
 * \param[in] lhs The message on the left hand size of the equality operator.
 * \param[in] rhs The message on the right hand size of the equality operator.
 * \return true if messages are equal, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_pick_and_place_msgs
bool
pick_and_place_msgs__action__ExecuteGraspMove_Result__are_equal(const pick_and_place_msgs__action__ExecuteGraspMove_Result * lhs, const pick_and_place_msgs__action__ExecuteGraspMove_Result * rhs);

/// Copy a action/ExecuteGraspMove message.
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
pick_and_place_msgs__action__ExecuteGraspMove_Result__copy(
  const pick_and_place_msgs__action__ExecuteGraspMove_Result * input,
  pick_and_place_msgs__action__ExecuteGraspMove_Result * output);

/// Initialize array of action/ExecuteGraspMove messages.
/**
 * It allocates the memory for the number of elements and calls
 * pick_and_place_msgs__action__ExecuteGraspMove_Result__init()
 * for each element of the array.
 * \param[in,out] array The allocated array pointer.
 * \param[in] size The size / capacity of the array.
 * \return true if initialization was successful, otherwise false
 * If the array pointer is valid and the size is zero it is guaranteed
 # to return true.
 */
ROSIDL_GENERATOR_C_PUBLIC_pick_and_place_msgs
bool
pick_and_place_msgs__action__ExecuteGraspMove_Result__Sequence__init(pick_and_place_msgs__action__ExecuteGraspMove_Result__Sequence * array, size_t size);

/// Finalize array of action/ExecuteGraspMove messages.
/**
 * It calls
 * pick_and_place_msgs__action__ExecuteGraspMove_Result__fini()
 * for each element of the array and frees the memory for the number of
 * elements.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_pick_and_place_msgs
void
pick_and_place_msgs__action__ExecuteGraspMove_Result__Sequence__fini(pick_and_place_msgs__action__ExecuteGraspMove_Result__Sequence * array);

/// Create array of action/ExecuteGraspMove messages.
/**
 * It allocates the memory for the array and calls
 * pick_and_place_msgs__action__ExecuteGraspMove_Result__Sequence__init().
 * \param[in] size The size / capacity of the array.
 * \return The pointer to the initialized array if successful, otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_pick_and_place_msgs
pick_and_place_msgs__action__ExecuteGraspMove_Result__Sequence *
pick_and_place_msgs__action__ExecuteGraspMove_Result__Sequence__create(size_t size);

/// Destroy array of action/ExecuteGraspMove messages.
/**
 * It calls
 * pick_and_place_msgs__action__ExecuteGraspMove_Result__Sequence__fini()
 * on the array,
 * and frees the memory of the array.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_pick_and_place_msgs
void
pick_and_place_msgs__action__ExecuteGraspMove_Result__Sequence__destroy(pick_and_place_msgs__action__ExecuteGraspMove_Result__Sequence * array);

/// Check for action/ExecuteGraspMove message array equality.
/**
 * \param[in] lhs The message array on the left hand size of the equality operator.
 * \param[in] rhs The message array on the right hand size of the equality operator.
 * \return true if message arrays are equal in size and content, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_pick_and_place_msgs
bool
pick_and_place_msgs__action__ExecuteGraspMove_Result__Sequence__are_equal(const pick_and_place_msgs__action__ExecuteGraspMove_Result__Sequence * lhs, const pick_and_place_msgs__action__ExecuteGraspMove_Result__Sequence * rhs);

/// Copy an array of action/ExecuteGraspMove messages.
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
pick_and_place_msgs__action__ExecuteGraspMove_Result__Sequence__copy(
  const pick_and_place_msgs__action__ExecuteGraspMove_Result__Sequence * input,
  pick_and_place_msgs__action__ExecuteGraspMove_Result__Sequence * output);

/// Initialize action/ExecuteGraspMove message.
/**
 * If the init function is called twice for the same message without
 * calling fini inbetween previously allocated memory will be leaked.
 * \param[in,out] msg The previously allocated message pointer.
 * Fields without a default value will not be initialized by this function.
 * You might want to call memset(msg, 0, sizeof(
 * pick_and_place_msgs__action__ExecuteGraspMove_Feedback
 * )) before or use
 * pick_and_place_msgs__action__ExecuteGraspMove_Feedback__create()
 * to allocate and initialize the message.
 * \return true if initialization was successful, otherwise false
 */
ROSIDL_GENERATOR_C_PUBLIC_pick_and_place_msgs
bool
pick_and_place_msgs__action__ExecuteGraspMove_Feedback__init(pick_and_place_msgs__action__ExecuteGraspMove_Feedback * msg);

/// Finalize action/ExecuteGraspMove message.
/**
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_pick_and_place_msgs
void
pick_and_place_msgs__action__ExecuteGraspMove_Feedback__fini(pick_and_place_msgs__action__ExecuteGraspMove_Feedback * msg);

/// Create action/ExecuteGraspMove message.
/**
 * It allocates the memory for the message, sets the memory to zero, and
 * calls
 * pick_and_place_msgs__action__ExecuteGraspMove_Feedback__init().
 * \return The pointer to the initialized message if successful,
 * otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_pick_and_place_msgs
pick_and_place_msgs__action__ExecuteGraspMove_Feedback *
pick_and_place_msgs__action__ExecuteGraspMove_Feedback__create();

/// Destroy action/ExecuteGraspMove message.
/**
 * It calls
 * pick_and_place_msgs__action__ExecuteGraspMove_Feedback__fini()
 * and frees the memory of the message.
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_pick_and_place_msgs
void
pick_and_place_msgs__action__ExecuteGraspMove_Feedback__destroy(pick_and_place_msgs__action__ExecuteGraspMove_Feedback * msg);

/// Check for action/ExecuteGraspMove message equality.
/**
 * \param[in] lhs The message on the left hand size of the equality operator.
 * \param[in] rhs The message on the right hand size of the equality operator.
 * \return true if messages are equal, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_pick_and_place_msgs
bool
pick_and_place_msgs__action__ExecuteGraspMove_Feedback__are_equal(const pick_and_place_msgs__action__ExecuteGraspMove_Feedback * lhs, const pick_and_place_msgs__action__ExecuteGraspMove_Feedback * rhs);

/// Copy a action/ExecuteGraspMove message.
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
pick_and_place_msgs__action__ExecuteGraspMove_Feedback__copy(
  const pick_and_place_msgs__action__ExecuteGraspMove_Feedback * input,
  pick_and_place_msgs__action__ExecuteGraspMove_Feedback * output);

/// Initialize array of action/ExecuteGraspMove messages.
/**
 * It allocates the memory for the number of elements and calls
 * pick_and_place_msgs__action__ExecuteGraspMove_Feedback__init()
 * for each element of the array.
 * \param[in,out] array The allocated array pointer.
 * \param[in] size The size / capacity of the array.
 * \return true if initialization was successful, otherwise false
 * If the array pointer is valid and the size is zero it is guaranteed
 # to return true.
 */
ROSIDL_GENERATOR_C_PUBLIC_pick_and_place_msgs
bool
pick_and_place_msgs__action__ExecuteGraspMove_Feedback__Sequence__init(pick_and_place_msgs__action__ExecuteGraspMove_Feedback__Sequence * array, size_t size);

/// Finalize array of action/ExecuteGraspMove messages.
/**
 * It calls
 * pick_and_place_msgs__action__ExecuteGraspMove_Feedback__fini()
 * for each element of the array and frees the memory for the number of
 * elements.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_pick_and_place_msgs
void
pick_and_place_msgs__action__ExecuteGraspMove_Feedback__Sequence__fini(pick_and_place_msgs__action__ExecuteGraspMove_Feedback__Sequence * array);

/// Create array of action/ExecuteGraspMove messages.
/**
 * It allocates the memory for the array and calls
 * pick_and_place_msgs__action__ExecuteGraspMove_Feedback__Sequence__init().
 * \param[in] size The size / capacity of the array.
 * \return The pointer to the initialized array if successful, otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_pick_and_place_msgs
pick_and_place_msgs__action__ExecuteGraspMove_Feedback__Sequence *
pick_and_place_msgs__action__ExecuteGraspMove_Feedback__Sequence__create(size_t size);

/// Destroy array of action/ExecuteGraspMove messages.
/**
 * It calls
 * pick_and_place_msgs__action__ExecuteGraspMove_Feedback__Sequence__fini()
 * on the array,
 * and frees the memory of the array.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_pick_and_place_msgs
void
pick_and_place_msgs__action__ExecuteGraspMove_Feedback__Sequence__destroy(pick_and_place_msgs__action__ExecuteGraspMove_Feedback__Sequence * array);

/// Check for action/ExecuteGraspMove message array equality.
/**
 * \param[in] lhs The message array on the left hand size of the equality operator.
 * \param[in] rhs The message array on the right hand size of the equality operator.
 * \return true if message arrays are equal in size and content, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_pick_and_place_msgs
bool
pick_and_place_msgs__action__ExecuteGraspMove_Feedback__Sequence__are_equal(const pick_and_place_msgs__action__ExecuteGraspMove_Feedback__Sequence * lhs, const pick_and_place_msgs__action__ExecuteGraspMove_Feedback__Sequence * rhs);

/// Copy an array of action/ExecuteGraspMove messages.
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
pick_and_place_msgs__action__ExecuteGraspMove_Feedback__Sequence__copy(
  const pick_and_place_msgs__action__ExecuteGraspMove_Feedback__Sequence * input,
  pick_and_place_msgs__action__ExecuteGraspMove_Feedback__Sequence * output);

/// Initialize action/ExecuteGraspMove message.
/**
 * If the init function is called twice for the same message without
 * calling fini inbetween previously allocated memory will be leaked.
 * \param[in,out] msg The previously allocated message pointer.
 * Fields without a default value will not be initialized by this function.
 * You might want to call memset(msg, 0, sizeof(
 * pick_and_place_msgs__action__ExecuteGraspMove_SendGoal_Request
 * )) before or use
 * pick_and_place_msgs__action__ExecuteGraspMove_SendGoal_Request__create()
 * to allocate and initialize the message.
 * \return true if initialization was successful, otherwise false
 */
ROSIDL_GENERATOR_C_PUBLIC_pick_and_place_msgs
bool
pick_and_place_msgs__action__ExecuteGraspMove_SendGoal_Request__init(pick_and_place_msgs__action__ExecuteGraspMove_SendGoal_Request * msg);

/// Finalize action/ExecuteGraspMove message.
/**
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_pick_and_place_msgs
void
pick_and_place_msgs__action__ExecuteGraspMove_SendGoal_Request__fini(pick_and_place_msgs__action__ExecuteGraspMove_SendGoal_Request * msg);

/// Create action/ExecuteGraspMove message.
/**
 * It allocates the memory for the message, sets the memory to zero, and
 * calls
 * pick_and_place_msgs__action__ExecuteGraspMove_SendGoal_Request__init().
 * \return The pointer to the initialized message if successful,
 * otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_pick_and_place_msgs
pick_and_place_msgs__action__ExecuteGraspMove_SendGoal_Request *
pick_and_place_msgs__action__ExecuteGraspMove_SendGoal_Request__create();

/// Destroy action/ExecuteGraspMove message.
/**
 * It calls
 * pick_and_place_msgs__action__ExecuteGraspMove_SendGoal_Request__fini()
 * and frees the memory of the message.
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_pick_and_place_msgs
void
pick_and_place_msgs__action__ExecuteGraspMove_SendGoal_Request__destroy(pick_and_place_msgs__action__ExecuteGraspMove_SendGoal_Request * msg);

/// Check for action/ExecuteGraspMove message equality.
/**
 * \param[in] lhs The message on the left hand size of the equality operator.
 * \param[in] rhs The message on the right hand size of the equality operator.
 * \return true if messages are equal, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_pick_and_place_msgs
bool
pick_and_place_msgs__action__ExecuteGraspMove_SendGoal_Request__are_equal(const pick_and_place_msgs__action__ExecuteGraspMove_SendGoal_Request * lhs, const pick_and_place_msgs__action__ExecuteGraspMove_SendGoal_Request * rhs);

/// Copy a action/ExecuteGraspMove message.
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
pick_and_place_msgs__action__ExecuteGraspMove_SendGoal_Request__copy(
  const pick_and_place_msgs__action__ExecuteGraspMove_SendGoal_Request * input,
  pick_and_place_msgs__action__ExecuteGraspMove_SendGoal_Request * output);

/// Initialize array of action/ExecuteGraspMove messages.
/**
 * It allocates the memory for the number of elements and calls
 * pick_and_place_msgs__action__ExecuteGraspMove_SendGoal_Request__init()
 * for each element of the array.
 * \param[in,out] array The allocated array pointer.
 * \param[in] size The size / capacity of the array.
 * \return true if initialization was successful, otherwise false
 * If the array pointer is valid and the size is zero it is guaranteed
 # to return true.
 */
ROSIDL_GENERATOR_C_PUBLIC_pick_and_place_msgs
bool
pick_and_place_msgs__action__ExecuteGraspMove_SendGoal_Request__Sequence__init(pick_and_place_msgs__action__ExecuteGraspMove_SendGoal_Request__Sequence * array, size_t size);

/// Finalize array of action/ExecuteGraspMove messages.
/**
 * It calls
 * pick_and_place_msgs__action__ExecuteGraspMove_SendGoal_Request__fini()
 * for each element of the array and frees the memory for the number of
 * elements.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_pick_and_place_msgs
void
pick_and_place_msgs__action__ExecuteGraspMove_SendGoal_Request__Sequence__fini(pick_and_place_msgs__action__ExecuteGraspMove_SendGoal_Request__Sequence * array);

/// Create array of action/ExecuteGraspMove messages.
/**
 * It allocates the memory for the array and calls
 * pick_and_place_msgs__action__ExecuteGraspMove_SendGoal_Request__Sequence__init().
 * \param[in] size The size / capacity of the array.
 * \return The pointer to the initialized array if successful, otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_pick_and_place_msgs
pick_and_place_msgs__action__ExecuteGraspMove_SendGoal_Request__Sequence *
pick_and_place_msgs__action__ExecuteGraspMove_SendGoal_Request__Sequence__create(size_t size);

/// Destroy array of action/ExecuteGraspMove messages.
/**
 * It calls
 * pick_and_place_msgs__action__ExecuteGraspMove_SendGoal_Request__Sequence__fini()
 * on the array,
 * and frees the memory of the array.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_pick_and_place_msgs
void
pick_and_place_msgs__action__ExecuteGraspMove_SendGoal_Request__Sequence__destroy(pick_and_place_msgs__action__ExecuteGraspMove_SendGoal_Request__Sequence * array);

/// Check for action/ExecuteGraspMove message array equality.
/**
 * \param[in] lhs The message array on the left hand size of the equality operator.
 * \param[in] rhs The message array on the right hand size of the equality operator.
 * \return true if message arrays are equal in size and content, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_pick_and_place_msgs
bool
pick_and_place_msgs__action__ExecuteGraspMove_SendGoal_Request__Sequence__are_equal(const pick_and_place_msgs__action__ExecuteGraspMove_SendGoal_Request__Sequence * lhs, const pick_and_place_msgs__action__ExecuteGraspMove_SendGoal_Request__Sequence * rhs);

/// Copy an array of action/ExecuteGraspMove messages.
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
pick_and_place_msgs__action__ExecuteGraspMove_SendGoal_Request__Sequence__copy(
  const pick_and_place_msgs__action__ExecuteGraspMove_SendGoal_Request__Sequence * input,
  pick_and_place_msgs__action__ExecuteGraspMove_SendGoal_Request__Sequence * output);

/// Initialize action/ExecuteGraspMove message.
/**
 * If the init function is called twice for the same message without
 * calling fini inbetween previously allocated memory will be leaked.
 * \param[in,out] msg The previously allocated message pointer.
 * Fields without a default value will not be initialized by this function.
 * You might want to call memset(msg, 0, sizeof(
 * pick_and_place_msgs__action__ExecuteGraspMove_SendGoal_Response
 * )) before or use
 * pick_and_place_msgs__action__ExecuteGraspMove_SendGoal_Response__create()
 * to allocate and initialize the message.
 * \return true if initialization was successful, otherwise false
 */
ROSIDL_GENERATOR_C_PUBLIC_pick_and_place_msgs
bool
pick_and_place_msgs__action__ExecuteGraspMove_SendGoal_Response__init(pick_and_place_msgs__action__ExecuteGraspMove_SendGoal_Response * msg);

/// Finalize action/ExecuteGraspMove message.
/**
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_pick_and_place_msgs
void
pick_and_place_msgs__action__ExecuteGraspMove_SendGoal_Response__fini(pick_and_place_msgs__action__ExecuteGraspMove_SendGoal_Response * msg);

/// Create action/ExecuteGraspMove message.
/**
 * It allocates the memory for the message, sets the memory to zero, and
 * calls
 * pick_and_place_msgs__action__ExecuteGraspMove_SendGoal_Response__init().
 * \return The pointer to the initialized message if successful,
 * otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_pick_and_place_msgs
pick_and_place_msgs__action__ExecuteGraspMove_SendGoal_Response *
pick_and_place_msgs__action__ExecuteGraspMove_SendGoal_Response__create();

/// Destroy action/ExecuteGraspMove message.
/**
 * It calls
 * pick_and_place_msgs__action__ExecuteGraspMove_SendGoal_Response__fini()
 * and frees the memory of the message.
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_pick_and_place_msgs
void
pick_and_place_msgs__action__ExecuteGraspMove_SendGoal_Response__destroy(pick_and_place_msgs__action__ExecuteGraspMove_SendGoal_Response * msg);

/// Check for action/ExecuteGraspMove message equality.
/**
 * \param[in] lhs The message on the left hand size of the equality operator.
 * \param[in] rhs The message on the right hand size of the equality operator.
 * \return true if messages are equal, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_pick_and_place_msgs
bool
pick_and_place_msgs__action__ExecuteGraspMove_SendGoal_Response__are_equal(const pick_and_place_msgs__action__ExecuteGraspMove_SendGoal_Response * lhs, const pick_and_place_msgs__action__ExecuteGraspMove_SendGoal_Response * rhs);

/// Copy a action/ExecuteGraspMove message.
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
pick_and_place_msgs__action__ExecuteGraspMove_SendGoal_Response__copy(
  const pick_and_place_msgs__action__ExecuteGraspMove_SendGoal_Response * input,
  pick_and_place_msgs__action__ExecuteGraspMove_SendGoal_Response * output);

/// Initialize array of action/ExecuteGraspMove messages.
/**
 * It allocates the memory for the number of elements and calls
 * pick_and_place_msgs__action__ExecuteGraspMove_SendGoal_Response__init()
 * for each element of the array.
 * \param[in,out] array The allocated array pointer.
 * \param[in] size The size / capacity of the array.
 * \return true if initialization was successful, otherwise false
 * If the array pointer is valid and the size is zero it is guaranteed
 # to return true.
 */
ROSIDL_GENERATOR_C_PUBLIC_pick_and_place_msgs
bool
pick_and_place_msgs__action__ExecuteGraspMove_SendGoal_Response__Sequence__init(pick_and_place_msgs__action__ExecuteGraspMove_SendGoal_Response__Sequence * array, size_t size);

/// Finalize array of action/ExecuteGraspMove messages.
/**
 * It calls
 * pick_and_place_msgs__action__ExecuteGraspMove_SendGoal_Response__fini()
 * for each element of the array and frees the memory for the number of
 * elements.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_pick_and_place_msgs
void
pick_and_place_msgs__action__ExecuteGraspMove_SendGoal_Response__Sequence__fini(pick_and_place_msgs__action__ExecuteGraspMove_SendGoal_Response__Sequence * array);

/// Create array of action/ExecuteGraspMove messages.
/**
 * It allocates the memory for the array and calls
 * pick_and_place_msgs__action__ExecuteGraspMove_SendGoal_Response__Sequence__init().
 * \param[in] size The size / capacity of the array.
 * \return The pointer to the initialized array if successful, otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_pick_and_place_msgs
pick_and_place_msgs__action__ExecuteGraspMove_SendGoal_Response__Sequence *
pick_and_place_msgs__action__ExecuteGraspMove_SendGoal_Response__Sequence__create(size_t size);

/// Destroy array of action/ExecuteGraspMove messages.
/**
 * It calls
 * pick_and_place_msgs__action__ExecuteGraspMove_SendGoal_Response__Sequence__fini()
 * on the array,
 * and frees the memory of the array.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_pick_and_place_msgs
void
pick_and_place_msgs__action__ExecuteGraspMove_SendGoal_Response__Sequence__destroy(pick_and_place_msgs__action__ExecuteGraspMove_SendGoal_Response__Sequence * array);

/// Check for action/ExecuteGraspMove message array equality.
/**
 * \param[in] lhs The message array on the left hand size of the equality operator.
 * \param[in] rhs The message array on the right hand size of the equality operator.
 * \return true if message arrays are equal in size and content, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_pick_and_place_msgs
bool
pick_and_place_msgs__action__ExecuteGraspMove_SendGoal_Response__Sequence__are_equal(const pick_and_place_msgs__action__ExecuteGraspMove_SendGoal_Response__Sequence * lhs, const pick_and_place_msgs__action__ExecuteGraspMove_SendGoal_Response__Sequence * rhs);

/// Copy an array of action/ExecuteGraspMove messages.
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
pick_and_place_msgs__action__ExecuteGraspMove_SendGoal_Response__Sequence__copy(
  const pick_and_place_msgs__action__ExecuteGraspMove_SendGoal_Response__Sequence * input,
  pick_and_place_msgs__action__ExecuteGraspMove_SendGoal_Response__Sequence * output);

/// Initialize action/ExecuteGraspMove message.
/**
 * If the init function is called twice for the same message without
 * calling fini inbetween previously allocated memory will be leaked.
 * \param[in,out] msg The previously allocated message pointer.
 * Fields without a default value will not be initialized by this function.
 * You might want to call memset(msg, 0, sizeof(
 * pick_and_place_msgs__action__ExecuteGraspMove_GetResult_Request
 * )) before or use
 * pick_and_place_msgs__action__ExecuteGraspMove_GetResult_Request__create()
 * to allocate and initialize the message.
 * \return true if initialization was successful, otherwise false
 */
ROSIDL_GENERATOR_C_PUBLIC_pick_and_place_msgs
bool
pick_and_place_msgs__action__ExecuteGraspMove_GetResult_Request__init(pick_and_place_msgs__action__ExecuteGraspMove_GetResult_Request * msg);

/// Finalize action/ExecuteGraspMove message.
/**
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_pick_and_place_msgs
void
pick_and_place_msgs__action__ExecuteGraspMove_GetResult_Request__fini(pick_and_place_msgs__action__ExecuteGraspMove_GetResult_Request * msg);

/// Create action/ExecuteGraspMove message.
/**
 * It allocates the memory for the message, sets the memory to zero, and
 * calls
 * pick_and_place_msgs__action__ExecuteGraspMove_GetResult_Request__init().
 * \return The pointer to the initialized message if successful,
 * otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_pick_and_place_msgs
pick_and_place_msgs__action__ExecuteGraspMove_GetResult_Request *
pick_and_place_msgs__action__ExecuteGraspMove_GetResult_Request__create();

/// Destroy action/ExecuteGraspMove message.
/**
 * It calls
 * pick_and_place_msgs__action__ExecuteGraspMove_GetResult_Request__fini()
 * and frees the memory of the message.
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_pick_and_place_msgs
void
pick_and_place_msgs__action__ExecuteGraspMove_GetResult_Request__destroy(pick_and_place_msgs__action__ExecuteGraspMove_GetResult_Request * msg);

/// Check for action/ExecuteGraspMove message equality.
/**
 * \param[in] lhs The message on the left hand size of the equality operator.
 * \param[in] rhs The message on the right hand size of the equality operator.
 * \return true if messages are equal, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_pick_and_place_msgs
bool
pick_and_place_msgs__action__ExecuteGraspMove_GetResult_Request__are_equal(const pick_and_place_msgs__action__ExecuteGraspMove_GetResult_Request * lhs, const pick_and_place_msgs__action__ExecuteGraspMove_GetResult_Request * rhs);

/// Copy a action/ExecuteGraspMove message.
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
pick_and_place_msgs__action__ExecuteGraspMove_GetResult_Request__copy(
  const pick_and_place_msgs__action__ExecuteGraspMove_GetResult_Request * input,
  pick_and_place_msgs__action__ExecuteGraspMove_GetResult_Request * output);

/// Initialize array of action/ExecuteGraspMove messages.
/**
 * It allocates the memory for the number of elements and calls
 * pick_and_place_msgs__action__ExecuteGraspMove_GetResult_Request__init()
 * for each element of the array.
 * \param[in,out] array The allocated array pointer.
 * \param[in] size The size / capacity of the array.
 * \return true if initialization was successful, otherwise false
 * If the array pointer is valid and the size is zero it is guaranteed
 # to return true.
 */
ROSIDL_GENERATOR_C_PUBLIC_pick_and_place_msgs
bool
pick_and_place_msgs__action__ExecuteGraspMove_GetResult_Request__Sequence__init(pick_and_place_msgs__action__ExecuteGraspMove_GetResult_Request__Sequence * array, size_t size);

/// Finalize array of action/ExecuteGraspMove messages.
/**
 * It calls
 * pick_and_place_msgs__action__ExecuteGraspMove_GetResult_Request__fini()
 * for each element of the array and frees the memory for the number of
 * elements.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_pick_and_place_msgs
void
pick_and_place_msgs__action__ExecuteGraspMove_GetResult_Request__Sequence__fini(pick_and_place_msgs__action__ExecuteGraspMove_GetResult_Request__Sequence * array);

/// Create array of action/ExecuteGraspMove messages.
/**
 * It allocates the memory for the array and calls
 * pick_and_place_msgs__action__ExecuteGraspMove_GetResult_Request__Sequence__init().
 * \param[in] size The size / capacity of the array.
 * \return The pointer to the initialized array if successful, otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_pick_and_place_msgs
pick_and_place_msgs__action__ExecuteGraspMove_GetResult_Request__Sequence *
pick_and_place_msgs__action__ExecuteGraspMove_GetResult_Request__Sequence__create(size_t size);

/// Destroy array of action/ExecuteGraspMove messages.
/**
 * It calls
 * pick_and_place_msgs__action__ExecuteGraspMove_GetResult_Request__Sequence__fini()
 * on the array,
 * and frees the memory of the array.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_pick_and_place_msgs
void
pick_and_place_msgs__action__ExecuteGraspMove_GetResult_Request__Sequence__destroy(pick_and_place_msgs__action__ExecuteGraspMove_GetResult_Request__Sequence * array);

/// Check for action/ExecuteGraspMove message array equality.
/**
 * \param[in] lhs The message array on the left hand size of the equality operator.
 * \param[in] rhs The message array on the right hand size of the equality operator.
 * \return true if message arrays are equal in size and content, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_pick_and_place_msgs
bool
pick_and_place_msgs__action__ExecuteGraspMove_GetResult_Request__Sequence__are_equal(const pick_and_place_msgs__action__ExecuteGraspMove_GetResult_Request__Sequence * lhs, const pick_and_place_msgs__action__ExecuteGraspMove_GetResult_Request__Sequence * rhs);

/// Copy an array of action/ExecuteGraspMove messages.
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
pick_and_place_msgs__action__ExecuteGraspMove_GetResult_Request__Sequence__copy(
  const pick_and_place_msgs__action__ExecuteGraspMove_GetResult_Request__Sequence * input,
  pick_and_place_msgs__action__ExecuteGraspMove_GetResult_Request__Sequence * output);

/// Initialize action/ExecuteGraspMove message.
/**
 * If the init function is called twice for the same message without
 * calling fini inbetween previously allocated memory will be leaked.
 * \param[in,out] msg The previously allocated message pointer.
 * Fields without a default value will not be initialized by this function.
 * You might want to call memset(msg, 0, sizeof(
 * pick_and_place_msgs__action__ExecuteGraspMove_GetResult_Response
 * )) before or use
 * pick_and_place_msgs__action__ExecuteGraspMove_GetResult_Response__create()
 * to allocate and initialize the message.
 * \return true if initialization was successful, otherwise false
 */
ROSIDL_GENERATOR_C_PUBLIC_pick_and_place_msgs
bool
pick_and_place_msgs__action__ExecuteGraspMove_GetResult_Response__init(pick_and_place_msgs__action__ExecuteGraspMove_GetResult_Response * msg);

/// Finalize action/ExecuteGraspMove message.
/**
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_pick_and_place_msgs
void
pick_and_place_msgs__action__ExecuteGraspMove_GetResult_Response__fini(pick_and_place_msgs__action__ExecuteGraspMove_GetResult_Response * msg);

/// Create action/ExecuteGraspMove message.
/**
 * It allocates the memory for the message, sets the memory to zero, and
 * calls
 * pick_and_place_msgs__action__ExecuteGraspMove_GetResult_Response__init().
 * \return The pointer to the initialized message if successful,
 * otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_pick_and_place_msgs
pick_and_place_msgs__action__ExecuteGraspMove_GetResult_Response *
pick_and_place_msgs__action__ExecuteGraspMove_GetResult_Response__create();

/// Destroy action/ExecuteGraspMove message.
/**
 * It calls
 * pick_and_place_msgs__action__ExecuteGraspMove_GetResult_Response__fini()
 * and frees the memory of the message.
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_pick_and_place_msgs
void
pick_and_place_msgs__action__ExecuteGraspMove_GetResult_Response__destroy(pick_and_place_msgs__action__ExecuteGraspMove_GetResult_Response * msg);

/// Check for action/ExecuteGraspMove message equality.
/**
 * \param[in] lhs The message on the left hand size of the equality operator.
 * \param[in] rhs The message on the right hand size of the equality operator.
 * \return true if messages are equal, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_pick_and_place_msgs
bool
pick_and_place_msgs__action__ExecuteGraspMove_GetResult_Response__are_equal(const pick_and_place_msgs__action__ExecuteGraspMove_GetResult_Response * lhs, const pick_and_place_msgs__action__ExecuteGraspMove_GetResult_Response * rhs);

/// Copy a action/ExecuteGraspMove message.
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
pick_and_place_msgs__action__ExecuteGraspMove_GetResult_Response__copy(
  const pick_and_place_msgs__action__ExecuteGraspMove_GetResult_Response * input,
  pick_and_place_msgs__action__ExecuteGraspMove_GetResult_Response * output);

/// Initialize array of action/ExecuteGraspMove messages.
/**
 * It allocates the memory for the number of elements and calls
 * pick_and_place_msgs__action__ExecuteGraspMove_GetResult_Response__init()
 * for each element of the array.
 * \param[in,out] array The allocated array pointer.
 * \param[in] size The size / capacity of the array.
 * \return true if initialization was successful, otherwise false
 * If the array pointer is valid and the size is zero it is guaranteed
 # to return true.
 */
ROSIDL_GENERATOR_C_PUBLIC_pick_and_place_msgs
bool
pick_and_place_msgs__action__ExecuteGraspMove_GetResult_Response__Sequence__init(pick_and_place_msgs__action__ExecuteGraspMove_GetResult_Response__Sequence * array, size_t size);

/// Finalize array of action/ExecuteGraspMove messages.
/**
 * It calls
 * pick_and_place_msgs__action__ExecuteGraspMove_GetResult_Response__fini()
 * for each element of the array and frees the memory for the number of
 * elements.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_pick_and_place_msgs
void
pick_and_place_msgs__action__ExecuteGraspMove_GetResult_Response__Sequence__fini(pick_and_place_msgs__action__ExecuteGraspMove_GetResult_Response__Sequence * array);

/// Create array of action/ExecuteGraspMove messages.
/**
 * It allocates the memory for the array and calls
 * pick_and_place_msgs__action__ExecuteGraspMove_GetResult_Response__Sequence__init().
 * \param[in] size The size / capacity of the array.
 * \return The pointer to the initialized array if successful, otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_pick_and_place_msgs
pick_and_place_msgs__action__ExecuteGraspMove_GetResult_Response__Sequence *
pick_and_place_msgs__action__ExecuteGraspMove_GetResult_Response__Sequence__create(size_t size);

/// Destroy array of action/ExecuteGraspMove messages.
/**
 * It calls
 * pick_and_place_msgs__action__ExecuteGraspMove_GetResult_Response__Sequence__fini()
 * on the array,
 * and frees the memory of the array.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_pick_and_place_msgs
void
pick_and_place_msgs__action__ExecuteGraspMove_GetResult_Response__Sequence__destroy(pick_and_place_msgs__action__ExecuteGraspMove_GetResult_Response__Sequence * array);

/// Check for action/ExecuteGraspMove message array equality.
/**
 * \param[in] lhs The message array on the left hand size of the equality operator.
 * \param[in] rhs The message array on the right hand size of the equality operator.
 * \return true if message arrays are equal in size and content, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_pick_and_place_msgs
bool
pick_and_place_msgs__action__ExecuteGraspMove_GetResult_Response__Sequence__are_equal(const pick_and_place_msgs__action__ExecuteGraspMove_GetResult_Response__Sequence * lhs, const pick_and_place_msgs__action__ExecuteGraspMove_GetResult_Response__Sequence * rhs);

/// Copy an array of action/ExecuteGraspMove messages.
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
pick_and_place_msgs__action__ExecuteGraspMove_GetResult_Response__Sequence__copy(
  const pick_and_place_msgs__action__ExecuteGraspMove_GetResult_Response__Sequence * input,
  pick_and_place_msgs__action__ExecuteGraspMove_GetResult_Response__Sequence * output);

/// Initialize action/ExecuteGraspMove message.
/**
 * If the init function is called twice for the same message without
 * calling fini inbetween previously allocated memory will be leaked.
 * \param[in,out] msg The previously allocated message pointer.
 * Fields without a default value will not be initialized by this function.
 * You might want to call memset(msg, 0, sizeof(
 * pick_and_place_msgs__action__ExecuteGraspMove_FeedbackMessage
 * )) before or use
 * pick_and_place_msgs__action__ExecuteGraspMove_FeedbackMessage__create()
 * to allocate and initialize the message.
 * \return true if initialization was successful, otherwise false
 */
ROSIDL_GENERATOR_C_PUBLIC_pick_and_place_msgs
bool
pick_and_place_msgs__action__ExecuteGraspMove_FeedbackMessage__init(pick_and_place_msgs__action__ExecuteGraspMove_FeedbackMessage * msg);

/// Finalize action/ExecuteGraspMove message.
/**
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_pick_and_place_msgs
void
pick_and_place_msgs__action__ExecuteGraspMove_FeedbackMessage__fini(pick_and_place_msgs__action__ExecuteGraspMove_FeedbackMessage * msg);

/// Create action/ExecuteGraspMove message.
/**
 * It allocates the memory for the message, sets the memory to zero, and
 * calls
 * pick_and_place_msgs__action__ExecuteGraspMove_FeedbackMessage__init().
 * \return The pointer to the initialized message if successful,
 * otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_pick_and_place_msgs
pick_and_place_msgs__action__ExecuteGraspMove_FeedbackMessage *
pick_and_place_msgs__action__ExecuteGraspMove_FeedbackMessage__create();

/// Destroy action/ExecuteGraspMove message.
/**
 * It calls
 * pick_and_place_msgs__action__ExecuteGraspMove_FeedbackMessage__fini()
 * and frees the memory of the message.
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_pick_and_place_msgs
void
pick_and_place_msgs__action__ExecuteGraspMove_FeedbackMessage__destroy(pick_and_place_msgs__action__ExecuteGraspMove_FeedbackMessage * msg);

/// Check for action/ExecuteGraspMove message equality.
/**
 * \param[in] lhs The message on the left hand size of the equality operator.
 * \param[in] rhs The message on the right hand size of the equality operator.
 * \return true if messages are equal, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_pick_and_place_msgs
bool
pick_and_place_msgs__action__ExecuteGraspMove_FeedbackMessage__are_equal(const pick_and_place_msgs__action__ExecuteGraspMove_FeedbackMessage * lhs, const pick_and_place_msgs__action__ExecuteGraspMove_FeedbackMessage * rhs);

/// Copy a action/ExecuteGraspMove message.
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
pick_and_place_msgs__action__ExecuteGraspMove_FeedbackMessage__copy(
  const pick_and_place_msgs__action__ExecuteGraspMove_FeedbackMessage * input,
  pick_and_place_msgs__action__ExecuteGraspMove_FeedbackMessage * output);

/// Initialize array of action/ExecuteGraspMove messages.
/**
 * It allocates the memory for the number of elements and calls
 * pick_and_place_msgs__action__ExecuteGraspMove_FeedbackMessage__init()
 * for each element of the array.
 * \param[in,out] array The allocated array pointer.
 * \param[in] size The size / capacity of the array.
 * \return true if initialization was successful, otherwise false
 * If the array pointer is valid and the size is zero it is guaranteed
 # to return true.
 */
ROSIDL_GENERATOR_C_PUBLIC_pick_and_place_msgs
bool
pick_and_place_msgs__action__ExecuteGraspMove_FeedbackMessage__Sequence__init(pick_and_place_msgs__action__ExecuteGraspMove_FeedbackMessage__Sequence * array, size_t size);

/// Finalize array of action/ExecuteGraspMove messages.
/**
 * It calls
 * pick_and_place_msgs__action__ExecuteGraspMove_FeedbackMessage__fini()
 * for each element of the array and frees the memory for the number of
 * elements.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_pick_and_place_msgs
void
pick_and_place_msgs__action__ExecuteGraspMove_FeedbackMessage__Sequence__fini(pick_and_place_msgs__action__ExecuteGraspMove_FeedbackMessage__Sequence * array);

/// Create array of action/ExecuteGraspMove messages.
/**
 * It allocates the memory for the array and calls
 * pick_and_place_msgs__action__ExecuteGraspMove_FeedbackMessage__Sequence__init().
 * \param[in] size The size / capacity of the array.
 * \return The pointer to the initialized array if successful, otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_pick_and_place_msgs
pick_and_place_msgs__action__ExecuteGraspMove_FeedbackMessage__Sequence *
pick_and_place_msgs__action__ExecuteGraspMove_FeedbackMessage__Sequence__create(size_t size);

/// Destroy array of action/ExecuteGraspMove messages.
/**
 * It calls
 * pick_and_place_msgs__action__ExecuteGraspMove_FeedbackMessage__Sequence__fini()
 * on the array,
 * and frees the memory of the array.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_pick_and_place_msgs
void
pick_and_place_msgs__action__ExecuteGraspMove_FeedbackMessage__Sequence__destroy(pick_and_place_msgs__action__ExecuteGraspMove_FeedbackMessage__Sequence * array);

/// Check for action/ExecuteGraspMove message array equality.
/**
 * \param[in] lhs The message array on the left hand size of the equality operator.
 * \param[in] rhs The message array on the right hand size of the equality operator.
 * \return true if message arrays are equal in size and content, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_pick_and_place_msgs
bool
pick_and_place_msgs__action__ExecuteGraspMove_FeedbackMessage__Sequence__are_equal(const pick_and_place_msgs__action__ExecuteGraspMove_FeedbackMessage__Sequence * lhs, const pick_and_place_msgs__action__ExecuteGraspMove_FeedbackMessage__Sequence * rhs);

/// Copy an array of action/ExecuteGraspMove messages.
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
pick_and_place_msgs__action__ExecuteGraspMove_FeedbackMessage__Sequence__copy(
  const pick_and_place_msgs__action__ExecuteGraspMove_FeedbackMessage__Sequence * input,
  pick_and_place_msgs__action__ExecuteGraspMove_FeedbackMessage__Sequence * output);

#ifdef __cplusplus
}
#endif

#endif  // PICK_AND_PLACE_MSGS__ACTION__DETAIL__EXECUTE_GRASP_MOVE__FUNCTIONS_H_
