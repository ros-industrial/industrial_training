# http://ros.org/doc/groovy/api/catkin/html/user_guide/supposed.html
cmake_minimum_required(VERSION 2.8.3)
project(motoman_driver)
# Load catkin and all dependencies required for this package
find_package(catkin REQUIRED COMPONENTS std_msgs roscpp actionlib
    trajectory_msgs actionlib_msgs control_msgs sensor_msgs urdf
    industrial_msgs industrial_robot_client industrial_utils simple_message motoman_msgs roslint)
find_package(Boost REQUIRED COMPONENTS system thread)

roslint_cpp()
#######################################
## Adding directories and definitions #
#######################################
include_directories(include MotoPlus inform ${Boost_INCLUDE_DIR} ${catkin_INCLUDE_DIRS})
add_definitions(-DLINUXSOCKETS=1)  #use linux sockets for communication
add_definitions(-DROS=1)

catkin_package(
    DEPENDS
    CATKIN_DEPENDS std_msgs roscpp actionlib trajectory_msgs actionlib_msgs
            control_msgs sensor_msgs simple_message urdf industrial_msgs
            industrial_robot_client motoman_msgs
    INCLUDE_DIRS include
    LIBRARIES
)


set(MSG_SRC_FILES
  src/simple_message/motoman_motion_ctrl.cpp
  src/simple_message/motoman_motion_ctrl_message.cpp
  src/simple_message/motoman_motion_reply.cpp
  src/simple_message/motoman_motion_reply_message.cpp)


set(CLIENT_SRC_FILES
  src/industrial_robot_client/joint_relay_handler.cpp
  src/industrial_robot_client/joint_feedback_relay_handler.cpp
  src/industrial_robot_client/joint_feedback_ex_relay_handler.cpp
  src/industrial_robot_client/robot_state_interface.cpp
  src/industrial_robot_client/joint_trajectory_interface.cpp
  src/industrial_robot_client/joint_trajectory_streamer.cpp
  src/simple_message/joint_feedback_ex.cpp
  src/simple_message/joint_traj_pt_full_ex.cpp
  src/simple_message/messages/joint_feedback_ex_message.cpp
  src/simple_message/messages/joint_traj_pt_full_ex_message.cpp)

###########
## Build ##
###########


#-------------------------------------------------------------
# fs100 uses same byte-ordering as most i386-based PCs

# Simple message library
add_library(motoman_simple_message ${MSG_SRC_FILES})
target_link_libraries(motoman_simple_message
  simple_message)

# Industrial client library
add_library(motoman_industrial_robot_client ${CLIENT_SRC_FILES})
target_link_libraries(motoman_industrial_robot_client
  industrial_robot_client
  industrial_utils)

# Robot state node
add_executable(motoman_robot_state
  src/robot_state_node.cpp)
target_link_libraries(motoman_robot_state
  motoman_simple_message
  motoman_industrial_robot_client
  ${catkin_LIBRARIES})
set_target_properties(motoman_robot_state
  PROPERTIES OUTPUT_NAME robot_state
  PREFIX "")

# Motion streaming node
add_executable(motoman_motion_streaming_interface
  src/joint_streaming_node.cpp
  src/joint_trajectory_streamer.cpp
  src/motion_ctrl.cpp)
target_link_libraries(motoman_motion_streaming_interface
  motoman_simple_message
  motoman_industrial_robot_client
  ${catkin_LIBRARIES})
set_target_properties(motoman_motion_streaming_interface
  PROPERTIES OUTPUT_NAME motion_streaming_interface
  PREFIX "")


#----------------------------------------------------------------
# FS100 uses opposite byte-ordering from most i386-based PCs
# Simple message library
add_library(motoman_simple_message_bswap ${MSG_SRC_FILES})
set_target_properties(motoman_simple_message_bswap PROPERTIES COMPILE_DEFINITIONS "BYTE_SWAPPING")
target_link_libraries(motoman_simple_message_bswap
  simple_message_bswap)

# Industrial client library
add_library(motoman_industrial_robot_client_bswap ${CLIENT_SRC_FILES})
set_target_properties(motoman_industrial_robot_client PROPERTIES COMPILE_DEFINITIONS "BYTE_SWAPPING")
target_link_libraries(motoman_industrial_robot_client_bswap
  industrial_robot_client_bswap
  industrial_utils)

# Robot state node
add_executable(motoman_robot_state_bswap
  src/robot_state_node.cpp)
target_link_libraries(motoman_robot_state_bswap
  motoman_simple_message_bswap
  motoman_industrial_robot_client_bswap
  ${catkin_LIBRARIES})
set_target_properties(motoman_robot_state_bswap
  PROPERTIES OUTPUT_NAME robot_state_bswap
  PREFIX "")

# Motion streaming node
add_executable(motoman_motion_streaming_interface_bswap
  src/joint_streaming_node.cpp
  src/joint_trajectory_streamer.cpp
  src/motion_ctrl.cpp)
target_link_libraries(motoman_motion_streaming_interface_bswap
  motoman_simple_message_bswap
  motoman_industrial_robot_client_bswap
  ${catkin_LIBRARIES})
set_target_properties(motoman_motion_streaming_interface_bswap
  PROPERTIES OUTPUT_NAME motion_streaming_interface_bswap
  PREFIX "")

add_executable(${PROJECT_NAME}_joint_trajectory_action
  src/joint_trajectory_action_node.cpp
  src/industrial_robot_client/joint_trajectory_action.cpp)
target_link_libraries(${PROJECT_NAME}_joint_trajectory_action
  industrial_robot_client ${catkin_LIBRARIES})
add_dependencies(${PROJECT_NAME}_joint_trajectory_action industrial_robot_client_gencpp)
#############
## Install ##
#############

# binaries
install(TARGETS
    motoman_robot_state
    motoman_motion_streaming_interface
    motoman_robot_state_bswap
    motoman_motion_streaming_interface_bswap
    ${PROJECT_NAME}_joint_trajectory_action

    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# libraries
install(TARGETS
    motoman_simple_message
    motoman_industrial_robot_client
    motoman_simple_message_bswap
    motoman_industrial_robot_client_bswap

    DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

# headers
install(DIRECTORY include/${PROJECT_NAME}/
    DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

# other files
foreach(dir Inform launch MotoPlus)
   install(DIRECTORY ${dir}/
      DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/${dir})
endforeach(dir)
