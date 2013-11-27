cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
#set(ROS_BUILD_TYPE RelWithDebInfo)

# Build static libs, to reduce dependency-chaining for industrial_robot_client
set(ROS_BUILD_STATIC_LIBS true)
set(ROS_BUILD_SHARED_LIBS false)

rosbuild_init()

# The simple_message library is designed to cross compile on Ubuntu
# and various robot controllers.  This requires conditionally compiling
# certain functions and headers.  The definition below enables compiling
# for a ROS node.
add_definitions(-DROS=1)  #build using ROS libraries
add_definitions(-DLINUXSOCKETS=1)  #use linux sockets for communication

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

#uncomment if you have defined messages
#rosbuild_genmsg()
#uncomment if you have defined services
#rosbuild_gensrv()

set(SRC_FILES src/byte_array.cpp
					src/simple_message.cpp
					src/smpl_msg_connection.cpp
					
					src/socket/simple_socket.cpp
					src/socket/udp_socket.cpp
					src/socket/udp_client.cpp
					src/socket/udp_server.cpp
					src/socket/tcp_socket.cpp
					src/socket/tcp_client.cpp
					src/socket/tcp_server.cpp
					
					src/message_handler.cpp
					src/message_manager.cpp
					src/ping_handler.cpp
					src/ping_message.cpp
					src/joint_data.cpp
					src/joint_feedback.cpp
					src/joint_traj_pt.cpp
					src/joint_traj_pt_full.cpp
					src/joint_traj.cpp
					src/robot_status.cpp
					
					src/messages/joint_message.cpp
					src/messages/joint_feedback_message.cpp
					src/messages/joint_traj_pt_message.cpp
					src/messages/joint_traj_pt_full_message.cpp
					src/messages/robot_status_message.cpp
					
					src/simple_comms_fault_handler.cpp)
					
set(UTEST_SRC_FILES test/utest.cpp
						test/utest_message.cpp)

# The simple message make file builds two libraries: simple_message and
# simple_message_byte_swapping.
#
# simple_message - is the default library.  This library should be used
# when the target for the simple message is the same endian (i.e. both
# big-endian or little-endian).  Intel based machines are little endian
#
# simple_message_byte_swapping - is an alternative library that can be used
# when the target for simple message is a DIFFERENT endian AND when the target
# target cannot perform byte swapping (as is the case for some industrial
# controllers).  This library performs byte swapping at the lowest load/unload
# levels.

# DEFAULT LIBRARY (SAME ENDIAN)
rosbuild_add_library(simple_message ${SRC_FILES})
rosbuild_add_compile_flags(simple_message "-fPIC") # for use in shared-libs

rosbuild_add_gtest(utest ${UTEST_SRC_FILES})
target_link_libraries(utest simple_message)

# ALTERNATIVE LIBRARY (DIFFERENT ENDIAN)
rosbuild_add_library(simple_message_bswap ${SRC_FILES})
set_target_properties(simple_message_bswap PROPERTIES COMPILE_DEFINITIONS "BYTE_SWAPPING")
rosbuild_add_compile_flags(simple_message_bswap "-fPIC") # for use in shared-libs

rosbuild_add_gtest(utest_byte_swapping ${UTEST_SRC_FILES})
target_link_libraries(utest_byte_swapping simple_message_bswap)

# ALTERNATIVE LIBRARY (64-bit floats)
rosbuild_add_library(simple_message_float64 ${SRC_FILES})
set_target_properties(simple_message_float64 PROPERTIES COMPILE_DEFINITIONS "FLOAT64")
rosbuild_add_compile_flags(simple_message_float64 "-fPIC") # for use in shared-libs

rosbuild_add_gtest(utest_float64 ${UTEST_SRC_FILES})
set_target_properties(utest_float64 PROPERTIES COMPILE_DEFINITIONS "FLOAT64")
target_link_libraries(utest_float64 simple_message_float64)
