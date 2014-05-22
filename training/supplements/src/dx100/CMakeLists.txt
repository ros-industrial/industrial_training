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
# TODO: Figure out how to export this definition from simple_message
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
#rosbuild_add_executable(joint_trajectory_action src/joint_trajectory_action.cpp)
rosbuild_add_library(motoman_lib 
	src/utils.cpp
	src/trajectory_job.cpp
	src/ros_conversion.cpp
	)
target_link_libraries(motoman_lib simple_message)

rosbuild_add_executable(system_inferface src/system_interface.cpp)
target_link_libraries(system_inferface simple_message)

rosbuild_add_executable(robot_state 
  src/robot_state.cpp)
target_link_libraries(robot_state industrial_robot_client)


rosbuild_add_executable(motion_interface
  src/motion_interface.cpp
  src/joint_trajectory_handler.cpp)
rosbuild_link_boost(motion_interface thread)
target_link_libraries(motion_interface simple_message)


rosbuild_add_executable(motion_download_interface
  src/motion_download_interface.cpp
  src/joint_trajectory_downloader.cpp
  )
target_link_libraries(motion_download_interface motoman_lib)

rosbuild_add_executable(joint_trajectory_action
  src/utils.cpp 
  src/joint_trajectory_action.cpp)
rosbuild_link_boost(joint_trajectory_action thread)
target_link_libraries(joint_trajectory_action simple_message)

rosbuild_add_gtest(utest test/utest.cpp)
target_link_libraries(utest motoman_lib)
