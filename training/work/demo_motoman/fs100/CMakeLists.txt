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
add_definitions(-DLINUXSOCKETS=1)  #use linux sockets for communication

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

#uncomment if you have defined messages
#rosbuild_genmsg()
#uncomment if you have defined services
#rosbuild_gensrv()

# NOTE: FS100 uses opposite byte-ordering from most i386-based PCs

rosbuild_add_library(fs100_simple_message
                     src/simple_message/motoman_motion_ctrl.cpp
                     src/simple_message/motoman_motion_ctrl_message.cpp
                     src/simple_message/motoman_motion_reply.cpp
                     src/simple_message/motoman_motion_reply_message.cpp
		     )
target_link_libraries(fs100_simple_message simple_message_bswap)
set_target_properties(fs100_simple_message PROPERTIES COMPILE_DEFINITIONS "BYTE_SWAPPING")

rosbuild_add_library(fs100_industrial_robot_client
                     src/industrial_robot_client/joint_relay_handler.cpp
                     src/industrial_robot_client/joint_feedback_relay_handler.cpp
                     src/industrial_robot_client/robot_state_interface.cpp
                     src/industrial_robot_client/joint_trajectory_interface.cpp
                     src/industrial_robot_client/joint_trajectory_streamer.cpp
                     )
target_link_libraries(fs100_industrial_robot_client industrial_robot_client_bswap)

rosbuild_add_executable(robot_state
                        src/fs100_robot_state_node.cpp)
target_link_libraries(robot_state
                      fs100_simple_message
                      fs100_industrial_robot_client)
                      
rosbuild_add_executable(motion_streaming_interface
                        src/fs100_joint_streaming_node.cpp
					    src/fs100_joint_trajectory_streamer.cpp
					    src/fs100_motion_ctrl.cpp)
target_link_libraries(motion_streaming_interface
					  fs100_simple_message
					  fs100_industrial_robot_client)

#common commands for building c++ executables and libraries
#rosbuild_add_library(${PROJECT_NAME} src/example.cpp)
#target_link_libraries(${PROJECT_NAME} another_library)
#rosbuild_add_boost_directories()
#rosbuild_link_boost(${PROJECT_NAME} thread)
#rosbuild_add_executable(example examples/example.cpp)
#target_link_libraries(example ${PROJECT_NAME})
