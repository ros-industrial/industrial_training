cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
#set(ROS_BUILD_TYPE RelWithDebInfo)

##rosbuild_add_generated_srvs(GetTargetPose.srv)
#ADD_DEFINITIONS("-std=c++0x")
rosbuild_init()

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

#uncomment if you have defined messages
##rosbuild_genmsg()
#uncomment if you have defined services
rosbuild_gensrv()

#common commands for building c++ executables and libraries
#rosbuild_add_library(${PROJECT_NAME} src/example.cpp)
#target_link_libraries(${PROJECT_NAME} another_library)
#rosbuild_add_boost_directories()
#rosbuild_link_boost(${PROJECT_NAME} thread)
#rosbuild_add_executable(example examples/example.cpp)
#target_link_libraries(example ${PROJECT_NAME})


rosbuild_add_executable(target_recognition_service src/services/target_recognition_service.cpp)

rosbuild_add_executable(pick_and_place_node 
	src/nodes/pick_and_place_node.cpp
	src/tasks/move_to_wait_position.cpp
	src/tasks/set_gripper.cpp
	src/tasks/reset_world.cpp
	src/tasks/set_attached_object.cpp
	src/tasks/detect_box_pick.cpp
	src/tasks/create_pick_moves.cpp
    src/tasks/create_place_moves.cpp
	src/tasks/pickup_box.cpp
	src/tasks/place_box.cpp	
	src/tasks/create_motion_plan.cpp
	src/utilities/pick_and_place_utilities.cpp)


	
