// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from pick_and_place_msgs:action/ExecuteGraspMove.idl
// generated code does not contain a copyright notice

#ifndef PICK_AND_PLACE_MSGS__ACTION__DETAIL__EXECUTE_GRASP_MOVE__STRUCT_H_
#define PICK_AND_PLACE_MSGS__ACTION__DETAIL__EXECUTE_GRASP_MOVE__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Constant 'PRE_GRASP'.
/**
  * the goal of this action
  * requests that the hand be set in the pre-grasp posture
 */
enum
{
  pick_and_place_msgs__action__ExecuteGraspMove_Goal__PRE_GRASP = 1l
};

/// Constant 'GRASP'.
/**
  * requests that the hand execute the actual grasp
 */
enum
{
  pick_and_place_msgs__action__ExecuteGraspMove_Goal__GRASP = 2l
};

/// Constant 'RELEASE'.
/**
  * requests that the hand open to release the object
 */
enum
{
  pick_and_place_msgs__action__ExecuteGraspMove_Goal__RELEASE = 3l
};

/// Struct defined in action/ExecuteGraspMove in the package pick_and_place_msgs.
typedef struct pick_and_place_msgs__action__ExecuteGraspMove_Goal
{
  int32_t goal;
  /// the max contact force to use (<=0 if no desired max)
  float max_contact_force;
} pick_and_place_msgs__action__ExecuteGraspMove_Goal;

// Struct for a sequence of pick_and_place_msgs__action__ExecuteGraspMove_Goal.
typedef struct pick_and_place_msgs__action__ExecuteGraspMove_Goal__Sequence
{
  pick_and_place_msgs__action__ExecuteGraspMove_Goal * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} pick_and_place_msgs__action__ExecuteGraspMove_Goal__Sequence;


// Constants defined in the message

/// Struct defined in action/ExecuteGraspMove in the package pick_and_place_msgs.
typedef struct pick_and_place_msgs__action__ExecuteGraspMove_Result
{
  bool success;
} pick_and_place_msgs__action__ExecuteGraspMove_Result;

// Struct for a sequence of pick_and_place_msgs__action__ExecuteGraspMove_Result.
typedef struct pick_and_place_msgs__action__ExecuteGraspMove_Result__Sequence
{
  pick_and_place_msgs__action__ExecuteGraspMove_Result * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} pick_and_place_msgs__action__ExecuteGraspMove_Result__Sequence;


// Constants defined in the message

/// Struct defined in action/ExecuteGraspMove in the package pick_and_place_msgs.
typedef struct pick_and_place_msgs__action__ExecuteGraspMove_Feedback
{
  uint8_t structure_needs_at_least_one_member;
} pick_and_place_msgs__action__ExecuteGraspMove_Feedback;

// Struct for a sequence of pick_and_place_msgs__action__ExecuteGraspMove_Feedback.
typedef struct pick_and_place_msgs__action__ExecuteGraspMove_Feedback__Sequence
{
  pick_and_place_msgs__action__ExecuteGraspMove_Feedback * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} pick_and_place_msgs__action__ExecuteGraspMove_Feedback__Sequence;


// Constants defined in the message

// Include directives for member types
// Member 'goal_id'
#include "unique_identifier_msgs/msg/detail/uuid__struct.h"
// Member 'goal'
#include "pick_and_place_msgs/action/detail/execute_grasp_move__struct.h"

/// Struct defined in action/ExecuteGraspMove in the package pick_and_place_msgs.
typedef struct pick_and_place_msgs__action__ExecuteGraspMove_SendGoal_Request
{
  unique_identifier_msgs__msg__UUID goal_id;
  pick_and_place_msgs__action__ExecuteGraspMove_Goal goal;
} pick_and_place_msgs__action__ExecuteGraspMove_SendGoal_Request;

// Struct for a sequence of pick_and_place_msgs__action__ExecuteGraspMove_SendGoal_Request.
typedef struct pick_and_place_msgs__action__ExecuteGraspMove_SendGoal_Request__Sequence
{
  pick_and_place_msgs__action__ExecuteGraspMove_SendGoal_Request * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} pick_and_place_msgs__action__ExecuteGraspMove_SendGoal_Request__Sequence;


// Constants defined in the message

// Include directives for member types
// Member 'stamp'
#include "builtin_interfaces/msg/detail/time__struct.h"

/// Struct defined in action/ExecuteGraspMove in the package pick_and_place_msgs.
typedef struct pick_and_place_msgs__action__ExecuteGraspMove_SendGoal_Response
{
  bool accepted;
  builtin_interfaces__msg__Time stamp;
} pick_and_place_msgs__action__ExecuteGraspMove_SendGoal_Response;

// Struct for a sequence of pick_and_place_msgs__action__ExecuteGraspMove_SendGoal_Response.
typedef struct pick_and_place_msgs__action__ExecuteGraspMove_SendGoal_Response__Sequence
{
  pick_and_place_msgs__action__ExecuteGraspMove_SendGoal_Response * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} pick_and_place_msgs__action__ExecuteGraspMove_SendGoal_Response__Sequence;


// Constants defined in the message

// Include directives for member types
// Member 'goal_id'
// already included above
// #include "unique_identifier_msgs/msg/detail/uuid__struct.h"

/// Struct defined in action/ExecuteGraspMove in the package pick_and_place_msgs.
typedef struct pick_and_place_msgs__action__ExecuteGraspMove_GetResult_Request
{
  unique_identifier_msgs__msg__UUID goal_id;
} pick_and_place_msgs__action__ExecuteGraspMove_GetResult_Request;

// Struct for a sequence of pick_and_place_msgs__action__ExecuteGraspMove_GetResult_Request.
typedef struct pick_and_place_msgs__action__ExecuteGraspMove_GetResult_Request__Sequence
{
  pick_and_place_msgs__action__ExecuteGraspMove_GetResult_Request * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} pick_and_place_msgs__action__ExecuteGraspMove_GetResult_Request__Sequence;


// Constants defined in the message

// Include directives for member types
// Member 'result'
// already included above
// #include "pick_and_place_msgs/action/detail/execute_grasp_move__struct.h"

/// Struct defined in action/ExecuteGraspMove in the package pick_and_place_msgs.
typedef struct pick_and_place_msgs__action__ExecuteGraspMove_GetResult_Response
{
  int8_t status;
  pick_and_place_msgs__action__ExecuteGraspMove_Result result;
} pick_and_place_msgs__action__ExecuteGraspMove_GetResult_Response;

// Struct for a sequence of pick_and_place_msgs__action__ExecuteGraspMove_GetResult_Response.
typedef struct pick_and_place_msgs__action__ExecuteGraspMove_GetResult_Response__Sequence
{
  pick_and_place_msgs__action__ExecuteGraspMove_GetResult_Response * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} pick_and_place_msgs__action__ExecuteGraspMove_GetResult_Response__Sequence;


// Constants defined in the message

// Include directives for member types
// Member 'goal_id'
// already included above
// #include "unique_identifier_msgs/msg/detail/uuid__struct.h"
// Member 'feedback'
// already included above
// #include "pick_and_place_msgs/action/detail/execute_grasp_move__struct.h"

/// Struct defined in action/ExecuteGraspMove in the package pick_and_place_msgs.
typedef struct pick_and_place_msgs__action__ExecuteGraspMove_FeedbackMessage
{
  unique_identifier_msgs__msg__UUID goal_id;
  pick_and_place_msgs__action__ExecuteGraspMove_Feedback feedback;
} pick_and_place_msgs__action__ExecuteGraspMove_FeedbackMessage;

// Struct for a sequence of pick_and_place_msgs__action__ExecuteGraspMove_FeedbackMessage.
typedef struct pick_and_place_msgs__action__ExecuteGraspMove_FeedbackMessage__Sequence
{
  pick_and_place_msgs__action__ExecuteGraspMove_FeedbackMessage * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} pick_and_place_msgs__action__ExecuteGraspMove_FeedbackMessage__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // PICK_AND_PLACE_MSGS__ACTION__DETAIL__EXECUTE_GRASP_MOVE__STRUCT_H_
