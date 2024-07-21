// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from pick_and_place_msgs:action/ExecuteGraspMove.idl
// generated code does not contain a copyright notice

#ifndef PICK_AND_PLACE_MSGS__ACTION__DETAIL__EXECUTE_GRASP_MOVE__BUILDER_HPP_
#define PICK_AND_PLACE_MSGS__ACTION__DETAIL__EXECUTE_GRASP_MOVE__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "pick_and_place_msgs/action/detail/execute_grasp_move__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace pick_and_place_msgs
{

namespace action
{

namespace builder
{

class Init_ExecuteGraspMove_Goal_max_contact_force
{
public:
  explicit Init_ExecuteGraspMove_Goal_max_contact_force(::pick_and_place_msgs::action::ExecuteGraspMove_Goal & msg)
  : msg_(msg)
  {}
  ::pick_and_place_msgs::action::ExecuteGraspMove_Goal max_contact_force(::pick_and_place_msgs::action::ExecuteGraspMove_Goal::_max_contact_force_type arg)
  {
    msg_.max_contact_force = std::move(arg);
    return std::move(msg_);
  }

private:
  ::pick_and_place_msgs::action::ExecuteGraspMove_Goal msg_;
};

class Init_ExecuteGraspMove_Goal_goal
{
public:
  Init_ExecuteGraspMove_Goal_goal()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_ExecuteGraspMove_Goal_max_contact_force goal(::pick_and_place_msgs::action::ExecuteGraspMove_Goal::_goal_type arg)
  {
    msg_.goal = std::move(arg);
    return Init_ExecuteGraspMove_Goal_max_contact_force(msg_);
  }

private:
  ::pick_and_place_msgs::action::ExecuteGraspMove_Goal msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::pick_and_place_msgs::action::ExecuteGraspMove_Goal>()
{
  return pick_and_place_msgs::action::builder::Init_ExecuteGraspMove_Goal_goal();
}

}  // namespace pick_and_place_msgs


namespace pick_and_place_msgs
{

namespace action
{

namespace builder
{

class Init_ExecuteGraspMove_Result_success
{
public:
  Init_ExecuteGraspMove_Result_success()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::pick_and_place_msgs::action::ExecuteGraspMove_Result success(::pick_and_place_msgs::action::ExecuteGraspMove_Result::_success_type arg)
  {
    msg_.success = std::move(arg);
    return std::move(msg_);
  }

private:
  ::pick_and_place_msgs::action::ExecuteGraspMove_Result msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::pick_and_place_msgs::action::ExecuteGraspMove_Result>()
{
  return pick_and_place_msgs::action::builder::Init_ExecuteGraspMove_Result_success();
}

}  // namespace pick_and_place_msgs


namespace pick_and_place_msgs
{

namespace action
{


}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::pick_and_place_msgs::action::ExecuteGraspMove_Feedback>()
{
  return ::pick_and_place_msgs::action::ExecuteGraspMove_Feedback(rosidl_runtime_cpp::MessageInitialization::ZERO);
}

}  // namespace pick_and_place_msgs


namespace pick_and_place_msgs
{

namespace action
{

namespace builder
{

class Init_ExecuteGraspMove_SendGoal_Request_goal
{
public:
  explicit Init_ExecuteGraspMove_SendGoal_Request_goal(::pick_and_place_msgs::action::ExecuteGraspMove_SendGoal_Request & msg)
  : msg_(msg)
  {}
  ::pick_and_place_msgs::action::ExecuteGraspMove_SendGoal_Request goal(::pick_and_place_msgs::action::ExecuteGraspMove_SendGoal_Request::_goal_type arg)
  {
    msg_.goal = std::move(arg);
    return std::move(msg_);
  }

private:
  ::pick_and_place_msgs::action::ExecuteGraspMove_SendGoal_Request msg_;
};

class Init_ExecuteGraspMove_SendGoal_Request_goal_id
{
public:
  Init_ExecuteGraspMove_SendGoal_Request_goal_id()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_ExecuteGraspMove_SendGoal_Request_goal goal_id(::pick_and_place_msgs::action::ExecuteGraspMove_SendGoal_Request::_goal_id_type arg)
  {
    msg_.goal_id = std::move(arg);
    return Init_ExecuteGraspMove_SendGoal_Request_goal(msg_);
  }

private:
  ::pick_and_place_msgs::action::ExecuteGraspMove_SendGoal_Request msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::pick_and_place_msgs::action::ExecuteGraspMove_SendGoal_Request>()
{
  return pick_and_place_msgs::action::builder::Init_ExecuteGraspMove_SendGoal_Request_goal_id();
}

}  // namespace pick_and_place_msgs


namespace pick_and_place_msgs
{

namespace action
{

namespace builder
{

class Init_ExecuteGraspMove_SendGoal_Response_stamp
{
public:
  explicit Init_ExecuteGraspMove_SendGoal_Response_stamp(::pick_and_place_msgs::action::ExecuteGraspMove_SendGoal_Response & msg)
  : msg_(msg)
  {}
  ::pick_and_place_msgs::action::ExecuteGraspMove_SendGoal_Response stamp(::pick_and_place_msgs::action::ExecuteGraspMove_SendGoal_Response::_stamp_type arg)
  {
    msg_.stamp = std::move(arg);
    return std::move(msg_);
  }

private:
  ::pick_and_place_msgs::action::ExecuteGraspMove_SendGoal_Response msg_;
};

class Init_ExecuteGraspMove_SendGoal_Response_accepted
{
public:
  Init_ExecuteGraspMove_SendGoal_Response_accepted()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_ExecuteGraspMove_SendGoal_Response_stamp accepted(::pick_and_place_msgs::action::ExecuteGraspMove_SendGoal_Response::_accepted_type arg)
  {
    msg_.accepted = std::move(arg);
    return Init_ExecuteGraspMove_SendGoal_Response_stamp(msg_);
  }

private:
  ::pick_and_place_msgs::action::ExecuteGraspMove_SendGoal_Response msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::pick_and_place_msgs::action::ExecuteGraspMove_SendGoal_Response>()
{
  return pick_and_place_msgs::action::builder::Init_ExecuteGraspMove_SendGoal_Response_accepted();
}

}  // namespace pick_and_place_msgs


namespace pick_and_place_msgs
{

namespace action
{

namespace builder
{

class Init_ExecuteGraspMove_GetResult_Request_goal_id
{
public:
  Init_ExecuteGraspMove_GetResult_Request_goal_id()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::pick_and_place_msgs::action::ExecuteGraspMove_GetResult_Request goal_id(::pick_and_place_msgs::action::ExecuteGraspMove_GetResult_Request::_goal_id_type arg)
  {
    msg_.goal_id = std::move(arg);
    return std::move(msg_);
  }

private:
  ::pick_and_place_msgs::action::ExecuteGraspMove_GetResult_Request msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::pick_and_place_msgs::action::ExecuteGraspMove_GetResult_Request>()
{
  return pick_and_place_msgs::action::builder::Init_ExecuteGraspMove_GetResult_Request_goal_id();
}

}  // namespace pick_and_place_msgs


namespace pick_and_place_msgs
{

namespace action
{

namespace builder
{

class Init_ExecuteGraspMove_GetResult_Response_result
{
public:
  explicit Init_ExecuteGraspMove_GetResult_Response_result(::pick_and_place_msgs::action::ExecuteGraspMove_GetResult_Response & msg)
  : msg_(msg)
  {}
  ::pick_and_place_msgs::action::ExecuteGraspMove_GetResult_Response result(::pick_and_place_msgs::action::ExecuteGraspMove_GetResult_Response::_result_type arg)
  {
    msg_.result = std::move(arg);
    return std::move(msg_);
  }

private:
  ::pick_and_place_msgs::action::ExecuteGraspMove_GetResult_Response msg_;
};

class Init_ExecuteGraspMove_GetResult_Response_status
{
public:
  Init_ExecuteGraspMove_GetResult_Response_status()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_ExecuteGraspMove_GetResult_Response_result status(::pick_and_place_msgs::action::ExecuteGraspMove_GetResult_Response::_status_type arg)
  {
    msg_.status = std::move(arg);
    return Init_ExecuteGraspMove_GetResult_Response_result(msg_);
  }

private:
  ::pick_and_place_msgs::action::ExecuteGraspMove_GetResult_Response msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::pick_and_place_msgs::action::ExecuteGraspMove_GetResult_Response>()
{
  return pick_and_place_msgs::action::builder::Init_ExecuteGraspMove_GetResult_Response_status();
}

}  // namespace pick_and_place_msgs


namespace pick_and_place_msgs
{

namespace action
{

namespace builder
{

class Init_ExecuteGraspMove_FeedbackMessage_feedback
{
public:
  explicit Init_ExecuteGraspMove_FeedbackMessage_feedback(::pick_and_place_msgs::action::ExecuteGraspMove_FeedbackMessage & msg)
  : msg_(msg)
  {}
  ::pick_and_place_msgs::action::ExecuteGraspMove_FeedbackMessage feedback(::pick_and_place_msgs::action::ExecuteGraspMove_FeedbackMessage::_feedback_type arg)
  {
    msg_.feedback = std::move(arg);
    return std::move(msg_);
  }

private:
  ::pick_and_place_msgs::action::ExecuteGraspMove_FeedbackMessage msg_;
};

class Init_ExecuteGraspMove_FeedbackMessage_goal_id
{
public:
  Init_ExecuteGraspMove_FeedbackMessage_goal_id()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_ExecuteGraspMove_FeedbackMessage_feedback goal_id(::pick_and_place_msgs::action::ExecuteGraspMove_FeedbackMessage::_goal_id_type arg)
  {
    msg_.goal_id = std::move(arg);
    return Init_ExecuteGraspMove_FeedbackMessage_feedback(msg_);
  }

private:
  ::pick_and_place_msgs::action::ExecuteGraspMove_FeedbackMessage msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::pick_and_place_msgs::action::ExecuteGraspMove_FeedbackMessage>()
{
  return pick_and_place_msgs::action::builder::Init_ExecuteGraspMove_FeedbackMessage_goal_id();
}

}  // namespace pick_and_place_msgs

#endif  // PICK_AND_PLACE_MSGS__ACTION__DETAIL__EXECUTE_GRASP_MOVE__BUILDER_HPP_
