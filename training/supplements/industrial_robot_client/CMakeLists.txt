cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
#set(ROS_BUILD_TYPE RelWithDebInfo)

rosbuild_init()

# The definition is copied from the CMakeList for the simple_message package.
add_definitions(-DROS=1)  #build using ROS libraries

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

#uncomment if you have defined messages
#rosbuild_genmsg()
#uncomment if you have defined services
#rosbuild_gensrv()

#common commands for building c++ executables and libraries
#rosbuild_add_library(${PROJECT_NAME} src/example.cpp)
#target_link_libraries(${PROJECT_NAME} another_library)
#rosbuild_add_boost_directories()
#rosbuild_link_boost(${PROJECT_NAME} thread)

set(SRC_FILES src/joint_relay_handler.cpp
			  src/robot_status_relay_handler.cpp
              src/joint_trajectory_downloader.cpp
              src/joint_trajectory_streamer.cpp
              src/joint_trajectory_interface.cpp
              src/robot_state_interface.cpp
              src/utils.cpp)
              
              

# This library depends on the simple_message library, which is available
# in two formats to support endian differences between robot & PC.
#
# As in simple_message, two libraries are created: one that does direct
# passthrough of binary numeric data to the robot controller, and one that
# uses byte-swapping to account for endian mis-matches.
#
# Due to rosmake dependency limitations, higher-level code must explicitly
# link to the desired industrial_robot_client library:
#    target_link_libraries(my_node industrial_robot_client)
#      or
#    target_link_libraries(my_node industrial_robot_client_bswap)

rosbuild_add_library(industrial_robot_client ${SRC_FILES})
target_link_libraries(industrial_robot_client simple_message)

rosbuild_add_library(industrial_robot_client_bswap ${SRC_FILES})
target_link_libraries(industrial_robot_client_bswap simple_message_bswap)


# The following executables(nodes) are for applications where the robot
# controller and pc have the same byte order (i.e. byte swapping NOT
# required)

rosbuild_add_executable(robot_state
  src/generic_robot_state_node.cpp)
target_link_libraries(robot_state industrial_robot_client)

rosbuild_add_executable(motion_streaming_interface
  src/generic_joint_streamer_node.cpp)
target_link_libraries(motion_streaming_interface industrial_robot_client)
rosbuild_link_boost(motion_streaming_interface thread)

rosbuild_add_executable(motion_download_interface
  src/generic_joint_downloader_node.cpp)
target_link_libraries(motion_download_interface industrial_robot_client)

# The following executables(nodes) are for applications where the robot
# controller and pc have different same byte order (i.e. byte swapping IS
# required)

rosbuild_add_executable(robot_state_bswap
  src/generic_robot_state_node.cpp)
target_link_libraries(robot_state_bswap industrial_robot_client_bswap)

rosbuild_add_executable(motion_streaming_interface_bswap
  src/generic_joint_streamer_node.cpp)
target_link_libraries(motion_streaming_interface_bswap industrial_robot_client_bswap)
rosbuild_link_boost(motion_streaming_interface thread)

rosbuild_add_executable(motion_download_interface_bswap
  src/generic_joint_downloader_node.cpp)
target_link_libraries(motion_download_interface_bswap industrial_robot_client_bswap)

# The following executables(nodes) interface with the robot controller
# at a higher level so there is no need to create two versions (one with
# byte swapping, one without)

rosbuild_add_executable(joint_trajectory_action 
  src/generic_joint_trajectory_action_node.cpp
  src/joint_trajectory_action.cpp)
target_link_libraries(joint_trajectory_action industrial_robot_client)
rosbuild_link_boost(joint_trajectory_action thread)

# Testing - Only performed on normal (non byte swapped library)
rosbuild_add_gtest(utest test/utest.cpp)
target_link_libraries(utest industrial_robot_client)


