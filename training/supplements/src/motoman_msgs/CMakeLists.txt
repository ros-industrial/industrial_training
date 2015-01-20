cmake_minimum_required(VERSION 2.8.3)
project(motoman_msgs)

find_package(catkin REQUIRED COMPONENTS std_msgs trajectory_msgs industrial_msgs genmsg message_generation)

add_message_files(
  FILES
  DynamicJointsGroup.msg
  DynamicJointPoint.msg
  DynamicJointTrajectory.msg
  DynamicJointState.msg
  DynamicJointTrajectoryFeedback.msg
  )

add_service_files(
  FILES
  CmdJointTrajectoryEx.srv
  )

generate_messages(
  DEPENDENCIES trajectory_msgs std_msgs industrial_msgs
)

catkin_package(
    CATKIN_DEPENDS message_runtime std_msgs trajectory_msgs industrial_msgs genmsg
)
