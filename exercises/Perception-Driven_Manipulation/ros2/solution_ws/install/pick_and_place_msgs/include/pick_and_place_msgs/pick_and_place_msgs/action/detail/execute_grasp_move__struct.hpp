// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from pick_and_place_msgs:action/ExecuteGraspMove.idl
// generated code does not contain a copyright notice

#ifndef PICK_AND_PLACE_MSGS__ACTION__DETAIL__EXECUTE_GRASP_MOVE__STRUCT_HPP_
#define PICK_AND_PLACE_MSGS__ACTION__DETAIL__EXECUTE_GRASP_MOVE__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__pick_and_place_msgs__action__ExecuteGraspMove_Goal __attribute__((deprecated))
#else
# define DEPRECATED__pick_and_place_msgs__action__ExecuteGraspMove_Goal __declspec(deprecated)
#endif

namespace pick_and_place_msgs
{

namespace action
{

// message struct
template<class ContainerAllocator>
struct ExecuteGraspMove_Goal_
{
  using Type = ExecuteGraspMove_Goal_<ContainerAllocator>;

  explicit ExecuteGraspMove_Goal_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->goal = 0l;
      this->max_contact_force = 0.0f;
    }
  }

  explicit ExecuteGraspMove_Goal_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->goal = 0l;
      this->max_contact_force = 0.0f;
    }
  }

  // field types and members
  using _goal_type =
    int32_t;
  _goal_type goal;
  using _max_contact_force_type =
    float;
  _max_contact_force_type max_contact_force;

  // setters for named parameter idiom
  Type & set__goal(
    const int32_t & _arg)
  {
    this->goal = _arg;
    return *this;
  }
  Type & set__max_contact_force(
    const float & _arg)
  {
    this->max_contact_force = _arg;
    return *this;
  }

  // constant declarations
  static constexpr int32_t PRE_GRASP =
    1;
  static constexpr int32_t GRASP =
    2;
  static constexpr int32_t RELEASE =
    3;

  // pointer types
  using RawPtr =
    pick_and_place_msgs::action::ExecuteGraspMove_Goal_<ContainerAllocator> *;
  using ConstRawPtr =
    const pick_and_place_msgs::action::ExecuteGraspMove_Goal_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<pick_and_place_msgs::action::ExecuteGraspMove_Goal_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<pick_and_place_msgs::action::ExecuteGraspMove_Goal_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      pick_and_place_msgs::action::ExecuteGraspMove_Goal_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<pick_and_place_msgs::action::ExecuteGraspMove_Goal_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      pick_and_place_msgs::action::ExecuteGraspMove_Goal_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<pick_and_place_msgs::action::ExecuteGraspMove_Goal_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<pick_and_place_msgs::action::ExecuteGraspMove_Goal_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<pick_and_place_msgs::action::ExecuteGraspMove_Goal_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__pick_and_place_msgs__action__ExecuteGraspMove_Goal
    std::shared_ptr<pick_and_place_msgs::action::ExecuteGraspMove_Goal_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__pick_and_place_msgs__action__ExecuteGraspMove_Goal
    std::shared_ptr<pick_and_place_msgs::action::ExecuteGraspMove_Goal_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const ExecuteGraspMove_Goal_ & other) const
  {
    if (this->goal != other.goal) {
      return false;
    }
    if (this->max_contact_force != other.max_contact_force) {
      return false;
    }
    return true;
  }
  bool operator!=(const ExecuteGraspMove_Goal_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct ExecuteGraspMove_Goal_

// alias to use template instance with default allocator
using ExecuteGraspMove_Goal =
  pick_and_place_msgs::action::ExecuteGraspMove_Goal_<std::allocator<void>>;

// constant definitions
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr int32_t ExecuteGraspMove_Goal_<ContainerAllocator>::PRE_GRASP;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr int32_t ExecuteGraspMove_Goal_<ContainerAllocator>::GRASP;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr int32_t ExecuteGraspMove_Goal_<ContainerAllocator>::RELEASE;
#endif  // __cplusplus < 201703L

}  // namespace action

}  // namespace pick_and_place_msgs


#ifndef _WIN32
# define DEPRECATED__pick_and_place_msgs__action__ExecuteGraspMove_Result __attribute__((deprecated))
#else
# define DEPRECATED__pick_and_place_msgs__action__ExecuteGraspMove_Result __declspec(deprecated)
#endif

namespace pick_and_place_msgs
{

namespace action
{

// message struct
template<class ContainerAllocator>
struct ExecuteGraspMove_Result_
{
  using Type = ExecuteGraspMove_Result_<ContainerAllocator>;

  explicit ExecuteGraspMove_Result_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->success = false;
    }
  }

  explicit ExecuteGraspMove_Result_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->success = false;
    }
  }

  // field types and members
  using _success_type =
    bool;
  _success_type success;

  // setters for named parameter idiom
  Type & set__success(
    const bool & _arg)
  {
    this->success = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    pick_and_place_msgs::action::ExecuteGraspMove_Result_<ContainerAllocator> *;
  using ConstRawPtr =
    const pick_and_place_msgs::action::ExecuteGraspMove_Result_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<pick_and_place_msgs::action::ExecuteGraspMove_Result_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<pick_and_place_msgs::action::ExecuteGraspMove_Result_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      pick_and_place_msgs::action::ExecuteGraspMove_Result_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<pick_and_place_msgs::action::ExecuteGraspMove_Result_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      pick_and_place_msgs::action::ExecuteGraspMove_Result_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<pick_and_place_msgs::action::ExecuteGraspMove_Result_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<pick_and_place_msgs::action::ExecuteGraspMove_Result_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<pick_and_place_msgs::action::ExecuteGraspMove_Result_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__pick_and_place_msgs__action__ExecuteGraspMove_Result
    std::shared_ptr<pick_and_place_msgs::action::ExecuteGraspMove_Result_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__pick_and_place_msgs__action__ExecuteGraspMove_Result
    std::shared_ptr<pick_and_place_msgs::action::ExecuteGraspMove_Result_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const ExecuteGraspMove_Result_ & other) const
  {
    if (this->success != other.success) {
      return false;
    }
    return true;
  }
  bool operator!=(const ExecuteGraspMove_Result_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct ExecuteGraspMove_Result_

// alias to use template instance with default allocator
using ExecuteGraspMove_Result =
  pick_and_place_msgs::action::ExecuteGraspMove_Result_<std::allocator<void>>;

// constant definitions

}  // namespace action

}  // namespace pick_and_place_msgs


#ifndef _WIN32
# define DEPRECATED__pick_and_place_msgs__action__ExecuteGraspMove_Feedback __attribute__((deprecated))
#else
# define DEPRECATED__pick_and_place_msgs__action__ExecuteGraspMove_Feedback __declspec(deprecated)
#endif

namespace pick_and_place_msgs
{

namespace action
{

// message struct
template<class ContainerAllocator>
struct ExecuteGraspMove_Feedback_
{
  using Type = ExecuteGraspMove_Feedback_<ContainerAllocator>;

  explicit ExecuteGraspMove_Feedback_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->structure_needs_at_least_one_member = 0;
    }
  }

  explicit ExecuteGraspMove_Feedback_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->structure_needs_at_least_one_member = 0;
    }
  }

  // field types and members
  using _structure_needs_at_least_one_member_type =
    uint8_t;
  _structure_needs_at_least_one_member_type structure_needs_at_least_one_member;


  // constant declarations

  // pointer types
  using RawPtr =
    pick_and_place_msgs::action::ExecuteGraspMove_Feedback_<ContainerAllocator> *;
  using ConstRawPtr =
    const pick_and_place_msgs::action::ExecuteGraspMove_Feedback_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<pick_and_place_msgs::action::ExecuteGraspMove_Feedback_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<pick_and_place_msgs::action::ExecuteGraspMove_Feedback_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      pick_and_place_msgs::action::ExecuteGraspMove_Feedback_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<pick_and_place_msgs::action::ExecuteGraspMove_Feedback_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      pick_and_place_msgs::action::ExecuteGraspMove_Feedback_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<pick_and_place_msgs::action::ExecuteGraspMove_Feedback_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<pick_and_place_msgs::action::ExecuteGraspMove_Feedback_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<pick_and_place_msgs::action::ExecuteGraspMove_Feedback_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__pick_and_place_msgs__action__ExecuteGraspMove_Feedback
    std::shared_ptr<pick_and_place_msgs::action::ExecuteGraspMove_Feedback_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__pick_and_place_msgs__action__ExecuteGraspMove_Feedback
    std::shared_ptr<pick_and_place_msgs::action::ExecuteGraspMove_Feedback_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const ExecuteGraspMove_Feedback_ & other) const
  {
    if (this->structure_needs_at_least_one_member != other.structure_needs_at_least_one_member) {
      return false;
    }
    return true;
  }
  bool operator!=(const ExecuteGraspMove_Feedback_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct ExecuteGraspMove_Feedback_

// alias to use template instance with default allocator
using ExecuteGraspMove_Feedback =
  pick_and_place_msgs::action::ExecuteGraspMove_Feedback_<std::allocator<void>>;

// constant definitions

}  // namespace action

}  // namespace pick_and_place_msgs


// Include directives for member types
// Member 'goal_id'
#include "unique_identifier_msgs/msg/detail/uuid__struct.hpp"
// Member 'goal'
#include "pick_and_place_msgs/action/detail/execute_grasp_move__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__pick_and_place_msgs__action__ExecuteGraspMove_SendGoal_Request __attribute__((deprecated))
#else
# define DEPRECATED__pick_and_place_msgs__action__ExecuteGraspMove_SendGoal_Request __declspec(deprecated)
#endif

namespace pick_and_place_msgs
{

namespace action
{

// message struct
template<class ContainerAllocator>
struct ExecuteGraspMove_SendGoal_Request_
{
  using Type = ExecuteGraspMove_SendGoal_Request_<ContainerAllocator>;

  explicit ExecuteGraspMove_SendGoal_Request_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : goal_id(_init),
    goal(_init)
  {
    (void)_init;
  }

  explicit ExecuteGraspMove_SendGoal_Request_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : goal_id(_alloc, _init),
    goal(_alloc, _init)
  {
    (void)_init;
  }

  // field types and members
  using _goal_id_type =
    unique_identifier_msgs::msg::UUID_<ContainerAllocator>;
  _goal_id_type goal_id;
  using _goal_type =
    pick_and_place_msgs::action::ExecuteGraspMove_Goal_<ContainerAllocator>;
  _goal_type goal;

  // setters for named parameter idiom
  Type & set__goal_id(
    const unique_identifier_msgs::msg::UUID_<ContainerAllocator> & _arg)
  {
    this->goal_id = _arg;
    return *this;
  }
  Type & set__goal(
    const pick_and_place_msgs::action::ExecuteGraspMove_Goal_<ContainerAllocator> & _arg)
  {
    this->goal = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    pick_and_place_msgs::action::ExecuteGraspMove_SendGoal_Request_<ContainerAllocator> *;
  using ConstRawPtr =
    const pick_and_place_msgs::action::ExecuteGraspMove_SendGoal_Request_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<pick_and_place_msgs::action::ExecuteGraspMove_SendGoal_Request_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<pick_and_place_msgs::action::ExecuteGraspMove_SendGoal_Request_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      pick_and_place_msgs::action::ExecuteGraspMove_SendGoal_Request_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<pick_and_place_msgs::action::ExecuteGraspMove_SendGoal_Request_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      pick_and_place_msgs::action::ExecuteGraspMove_SendGoal_Request_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<pick_and_place_msgs::action::ExecuteGraspMove_SendGoal_Request_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<pick_and_place_msgs::action::ExecuteGraspMove_SendGoal_Request_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<pick_and_place_msgs::action::ExecuteGraspMove_SendGoal_Request_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__pick_and_place_msgs__action__ExecuteGraspMove_SendGoal_Request
    std::shared_ptr<pick_and_place_msgs::action::ExecuteGraspMove_SendGoal_Request_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__pick_and_place_msgs__action__ExecuteGraspMove_SendGoal_Request
    std::shared_ptr<pick_and_place_msgs::action::ExecuteGraspMove_SendGoal_Request_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const ExecuteGraspMove_SendGoal_Request_ & other) const
  {
    if (this->goal_id != other.goal_id) {
      return false;
    }
    if (this->goal != other.goal) {
      return false;
    }
    return true;
  }
  bool operator!=(const ExecuteGraspMove_SendGoal_Request_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct ExecuteGraspMove_SendGoal_Request_

// alias to use template instance with default allocator
using ExecuteGraspMove_SendGoal_Request =
  pick_and_place_msgs::action::ExecuteGraspMove_SendGoal_Request_<std::allocator<void>>;

// constant definitions

}  // namespace action

}  // namespace pick_and_place_msgs


// Include directives for member types
// Member 'stamp'
#include "builtin_interfaces/msg/detail/time__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__pick_and_place_msgs__action__ExecuteGraspMove_SendGoal_Response __attribute__((deprecated))
#else
# define DEPRECATED__pick_and_place_msgs__action__ExecuteGraspMove_SendGoal_Response __declspec(deprecated)
#endif

namespace pick_and_place_msgs
{

namespace action
{

// message struct
template<class ContainerAllocator>
struct ExecuteGraspMove_SendGoal_Response_
{
  using Type = ExecuteGraspMove_SendGoal_Response_<ContainerAllocator>;

  explicit ExecuteGraspMove_SendGoal_Response_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : stamp(_init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->accepted = false;
    }
  }

  explicit ExecuteGraspMove_SendGoal_Response_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : stamp(_alloc, _init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->accepted = false;
    }
  }

  // field types and members
  using _accepted_type =
    bool;
  _accepted_type accepted;
  using _stamp_type =
    builtin_interfaces::msg::Time_<ContainerAllocator>;
  _stamp_type stamp;

  // setters for named parameter idiom
  Type & set__accepted(
    const bool & _arg)
  {
    this->accepted = _arg;
    return *this;
  }
  Type & set__stamp(
    const builtin_interfaces::msg::Time_<ContainerAllocator> & _arg)
  {
    this->stamp = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    pick_and_place_msgs::action::ExecuteGraspMove_SendGoal_Response_<ContainerAllocator> *;
  using ConstRawPtr =
    const pick_and_place_msgs::action::ExecuteGraspMove_SendGoal_Response_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<pick_and_place_msgs::action::ExecuteGraspMove_SendGoal_Response_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<pick_and_place_msgs::action::ExecuteGraspMove_SendGoal_Response_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      pick_and_place_msgs::action::ExecuteGraspMove_SendGoal_Response_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<pick_and_place_msgs::action::ExecuteGraspMove_SendGoal_Response_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      pick_and_place_msgs::action::ExecuteGraspMove_SendGoal_Response_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<pick_and_place_msgs::action::ExecuteGraspMove_SendGoal_Response_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<pick_and_place_msgs::action::ExecuteGraspMove_SendGoal_Response_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<pick_and_place_msgs::action::ExecuteGraspMove_SendGoal_Response_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__pick_and_place_msgs__action__ExecuteGraspMove_SendGoal_Response
    std::shared_ptr<pick_and_place_msgs::action::ExecuteGraspMove_SendGoal_Response_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__pick_and_place_msgs__action__ExecuteGraspMove_SendGoal_Response
    std::shared_ptr<pick_and_place_msgs::action::ExecuteGraspMove_SendGoal_Response_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const ExecuteGraspMove_SendGoal_Response_ & other) const
  {
    if (this->accepted != other.accepted) {
      return false;
    }
    if (this->stamp != other.stamp) {
      return false;
    }
    return true;
  }
  bool operator!=(const ExecuteGraspMove_SendGoal_Response_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct ExecuteGraspMove_SendGoal_Response_

// alias to use template instance with default allocator
using ExecuteGraspMove_SendGoal_Response =
  pick_and_place_msgs::action::ExecuteGraspMove_SendGoal_Response_<std::allocator<void>>;

// constant definitions

}  // namespace action

}  // namespace pick_and_place_msgs

namespace pick_and_place_msgs
{

namespace action
{

struct ExecuteGraspMove_SendGoal
{
  using Request = pick_and_place_msgs::action::ExecuteGraspMove_SendGoal_Request;
  using Response = pick_and_place_msgs::action::ExecuteGraspMove_SendGoal_Response;
};

}  // namespace action

}  // namespace pick_and_place_msgs


// Include directives for member types
// Member 'goal_id'
// already included above
// #include "unique_identifier_msgs/msg/detail/uuid__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__pick_and_place_msgs__action__ExecuteGraspMove_GetResult_Request __attribute__((deprecated))
#else
# define DEPRECATED__pick_and_place_msgs__action__ExecuteGraspMove_GetResult_Request __declspec(deprecated)
#endif

namespace pick_and_place_msgs
{

namespace action
{

// message struct
template<class ContainerAllocator>
struct ExecuteGraspMove_GetResult_Request_
{
  using Type = ExecuteGraspMove_GetResult_Request_<ContainerAllocator>;

  explicit ExecuteGraspMove_GetResult_Request_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : goal_id(_init)
  {
    (void)_init;
  }

  explicit ExecuteGraspMove_GetResult_Request_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : goal_id(_alloc, _init)
  {
    (void)_init;
  }

  // field types and members
  using _goal_id_type =
    unique_identifier_msgs::msg::UUID_<ContainerAllocator>;
  _goal_id_type goal_id;

  // setters for named parameter idiom
  Type & set__goal_id(
    const unique_identifier_msgs::msg::UUID_<ContainerAllocator> & _arg)
  {
    this->goal_id = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    pick_and_place_msgs::action::ExecuteGraspMove_GetResult_Request_<ContainerAllocator> *;
  using ConstRawPtr =
    const pick_and_place_msgs::action::ExecuteGraspMove_GetResult_Request_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<pick_and_place_msgs::action::ExecuteGraspMove_GetResult_Request_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<pick_and_place_msgs::action::ExecuteGraspMove_GetResult_Request_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      pick_and_place_msgs::action::ExecuteGraspMove_GetResult_Request_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<pick_and_place_msgs::action::ExecuteGraspMove_GetResult_Request_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      pick_and_place_msgs::action::ExecuteGraspMove_GetResult_Request_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<pick_and_place_msgs::action::ExecuteGraspMove_GetResult_Request_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<pick_and_place_msgs::action::ExecuteGraspMove_GetResult_Request_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<pick_and_place_msgs::action::ExecuteGraspMove_GetResult_Request_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__pick_and_place_msgs__action__ExecuteGraspMove_GetResult_Request
    std::shared_ptr<pick_and_place_msgs::action::ExecuteGraspMove_GetResult_Request_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__pick_and_place_msgs__action__ExecuteGraspMove_GetResult_Request
    std::shared_ptr<pick_and_place_msgs::action::ExecuteGraspMove_GetResult_Request_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const ExecuteGraspMove_GetResult_Request_ & other) const
  {
    if (this->goal_id != other.goal_id) {
      return false;
    }
    return true;
  }
  bool operator!=(const ExecuteGraspMove_GetResult_Request_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct ExecuteGraspMove_GetResult_Request_

// alias to use template instance with default allocator
using ExecuteGraspMove_GetResult_Request =
  pick_and_place_msgs::action::ExecuteGraspMove_GetResult_Request_<std::allocator<void>>;

// constant definitions

}  // namespace action

}  // namespace pick_and_place_msgs


// Include directives for member types
// Member 'result'
// already included above
// #include "pick_and_place_msgs/action/detail/execute_grasp_move__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__pick_and_place_msgs__action__ExecuteGraspMove_GetResult_Response __attribute__((deprecated))
#else
# define DEPRECATED__pick_and_place_msgs__action__ExecuteGraspMove_GetResult_Response __declspec(deprecated)
#endif

namespace pick_and_place_msgs
{

namespace action
{

// message struct
template<class ContainerAllocator>
struct ExecuteGraspMove_GetResult_Response_
{
  using Type = ExecuteGraspMove_GetResult_Response_<ContainerAllocator>;

  explicit ExecuteGraspMove_GetResult_Response_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : result(_init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->status = 0;
    }
  }

  explicit ExecuteGraspMove_GetResult_Response_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : result(_alloc, _init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->status = 0;
    }
  }

  // field types and members
  using _status_type =
    int8_t;
  _status_type status;
  using _result_type =
    pick_and_place_msgs::action::ExecuteGraspMove_Result_<ContainerAllocator>;
  _result_type result;

  // setters for named parameter idiom
  Type & set__status(
    const int8_t & _arg)
  {
    this->status = _arg;
    return *this;
  }
  Type & set__result(
    const pick_and_place_msgs::action::ExecuteGraspMove_Result_<ContainerAllocator> & _arg)
  {
    this->result = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    pick_and_place_msgs::action::ExecuteGraspMove_GetResult_Response_<ContainerAllocator> *;
  using ConstRawPtr =
    const pick_and_place_msgs::action::ExecuteGraspMove_GetResult_Response_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<pick_and_place_msgs::action::ExecuteGraspMove_GetResult_Response_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<pick_and_place_msgs::action::ExecuteGraspMove_GetResult_Response_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      pick_and_place_msgs::action::ExecuteGraspMove_GetResult_Response_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<pick_and_place_msgs::action::ExecuteGraspMove_GetResult_Response_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      pick_and_place_msgs::action::ExecuteGraspMove_GetResult_Response_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<pick_and_place_msgs::action::ExecuteGraspMove_GetResult_Response_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<pick_and_place_msgs::action::ExecuteGraspMove_GetResult_Response_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<pick_and_place_msgs::action::ExecuteGraspMove_GetResult_Response_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__pick_and_place_msgs__action__ExecuteGraspMove_GetResult_Response
    std::shared_ptr<pick_and_place_msgs::action::ExecuteGraspMove_GetResult_Response_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__pick_and_place_msgs__action__ExecuteGraspMove_GetResult_Response
    std::shared_ptr<pick_and_place_msgs::action::ExecuteGraspMove_GetResult_Response_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const ExecuteGraspMove_GetResult_Response_ & other) const
  {
    if (this->status != other.status) {
      return false;
    }
    if (this->result != other.result) {
      return false;
    }
    return true;
  }
  bool operator!=(const ExecuteGraspMove_GetResult_Response_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct ExecuteGraspMove_GetResult_Response_

// alias to use template instance with default allocator
using ExecuteGraspMove_GetResult_Response =
  pick_and_place_msgs::action::ExecuteGraspMove_GetResult_Response_<std::allocator<void>>;

// constant definitions

}  // namespace action

}  // namespace pick_and_place_msgs

namespace pick_and_place_msgs
{

namespace action
{

struct ExecuteGraspMove_GetResult
{
  using Request = pick_and_place_msgs::action::ExecuteGraspMove_GetResult_Request;
  using Response = pick_and_place_msgs::action::ExecuteGraspMove_GetResult_Response;
};

}  // namespace action

}  // namespace pick_and_place_msgs


// Include directives for member types
// Member 'goal_id'
// already included above
// #include "unique_identifier_msgs/msg/detail/uuid__struct.hpp"
// Member 'feedback'
// already included above
// #include "pick_and_place_msgs/action/detail/execute_grasp_move__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__pick_and_place_msgs__action__ExecuteGraspMove_FeedbackMessage __attribute__((deprecated))
#else
# define DEPRECATED__pick_and_place_msgs__action__ExecuteGraspMove_FeedbackMessage __declspec(deprecated)
#endif

namespace pick_and_place_msgs
{

namespace action
{

// message struct
template<class ContainerAllocator>
struct ExecuteGraspMove_FeedbackMessage_
{
  using Type = ExecuteGraspMove_FeedbackMessage_<ContainerAllocator>;

  explicit ExecuteGraspMove_FeedbackMessage_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : goal_id(_init),
    feedback(_init)
  {
    (void)_init;
  }

  explicit ExecuteGraspMove_FeedbackMessage_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : goal_id(_alloc, _init),
    feedback(_alloc, _init)
  {
    (void)_init;
  }

  // field types and members
  using _goal_id_type =
    unique_identifier_msgs::msg::UUID_<ContainerAllocator>;
  _goal_id_type goal_id;
  using _feedback_type =
    pick_and_place_msgs::action::ExecuteGraspMove_Feedback_<ContainerAllocator>;
  _feedback_type feedback;

  // setters for named parameter idiom
  Type & set__goal_id(
    const unique_identifier_msgs::msg::UUID_<ContainerAllocator> & _arg)
  {
    this->goal_id = _arg;
    return *this;
  }
  Type & set__feedback(
    const pick_and_place_msgs::action::ExecuteGraspMove_Feedback_<ContainerAllocator> & _arg)
  {
    this->feedback = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    pick_and_place_msgs::action::ExecuteGraspMove_FeedbackMessage_<ContainerAllocator> *;
  using ConstRawPtr =
    const pick_and_place_msgs::action::ExecuteGraspMove_FeedbackMessage_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<pick_and_place_msgs::action::ExecuteGraspMove_FeedbackMessage_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<pick_and_place_msgs::action::ExecuteGraspMove_FeedbackMessage_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      pick_and_place_msgs::action::ExecuteGraspMove_FeedbackMessage_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<pick_and_place_msgs::action::ExecuteGraspMove_FeedbackMessage_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      pick_and_place_msgs::action::ExecuteGraspMove_FeedbackMessage_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<pick_and_place_msgs::action::ExecuteGraspMove_FeedbackMessage_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<pick_and_place_msgs::action::ExecuteGraspMove_FeedbackMessage_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<pick_and_place_msgs::action::ExecuteGraspMove_FeedbackMessage_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__pick_and_place_msgs__action__ExecuteGraspMove_FeedbackMessage
    std::shared_ptr<pick_and_place_msgs::action::ExecuteGraspMove_FeedbackMessage_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__pick_and_place_msgs__action__ExecuteGraspMove_FeedbackMessage
    std::shared_ptr<pick_and_place_msgs::action::ExecuteGraspMove_FeedbackMessage_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const ExecuteGraspMove_FeedbackMessage_ & other) const
  {
    if (this->goal_id != other.goal_id) {
      return false;
    }
    if (this->feedback != other.feedback) {
      return false;
    }
    return true;
  }
  bool operator!=(const ExecuteGraspMove_FeedbackMessage_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct ExecuteGraspMove_FeedbackMessage_

// alias to use template instance with default allocator
using ExecuteGraspMove_FeedbackMessage =
  pick_and_place_msgs::action::ExecuteGraspMove_FeedbackMessage_<std::allocator<void>>;

// constant definitions

}  // namespace action

}  // namespace pick_and_place_msgs

#include "action_msgs/srv/cancel_goal.hpp"
#include "action_msgs/msg/goal_info.hpp"
#include "action_msgs/msg/goal_status_array.hpp"

namespace pick_and_place_msgs
{

namespace action
{

struct ExecuteGraspMove
{
  /// The goal message defined in the action definition.
  using Goal = pick_and_place_msgs::action::ExecuteGraspMove_Goal;
  /// The result message defined in the action definition.
  using Result = pick_and_place_msgs::action::ExecuteGraspMove_Result;
  /// The feedback message defined in the action definition.
  using Feedback = pick_and_place_msgs::action::ExecuteGraspMove_Feedback;

  struct Impl
  {
    /// The send_goal service using a wrapped version of the goal message as a request.
    using SendGoalService = pick_and_place_msgs::action::ExecuteGraspMove_SendGoal;
    /// The get_result service using a wrapped version of the result message as a response.
    using GetResultService = pick_and_place_msgs::action::ExecuteGraspMove_GetResult;
    /// The feedback message with generic fields which wraps the feedback message.
    using FeedbackMessage = pick_and_place_msgs::action::ExecuteGraspMove_FeedbackMessage;

    /// The generic service to cancel a goal.
    using CancelGoalService = action_msgs::srv::CancelGoal;
    /// The generic message for the status of a goal.
    using GoalStatusMessage = action_msgs::msg::GoalStatusArray;
  };
};

typedef struct ExecuteGraspMove ExecuteGraspMove;

}  // namespace action

}  // namespace pick_and_place_msgs

#endif  // PICK_AND_PLACE_MSGS__ACTION__DETAIL__EXECUTE_GRASP_MOVE__STRUCT_HPP_
