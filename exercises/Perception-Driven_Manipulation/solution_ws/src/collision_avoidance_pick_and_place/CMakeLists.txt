cmake_minimum_required(VERSION 3.5)
project(collision_avoidance_pick_and_place)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

# Disable Wredundant-decls warnings since rosidl generates redundant function
#  declarations
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  set(CMAKE_CXX_FLAGS 
    "${CMAKE_CXX_FLAGS} -Wall -Wextra -Wpedantic -Wno-redundant-decls")
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(shape_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(pick_and_place_msgs REQUIRED)
find_package(moveit_common REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)

find_package(Boost REQUIRED COMPONENTS system)

#######################################
## Declare ROS messages and services ##
#######################################
#rosidl_generate_interfaces(collision_avoidance_pick_and_place
#  srv/GetTargetPose.srv
#  DEPENDENCIES builtin_interfaces shape_msgs geometry_msgs
#)
#ament_export_dependencies(rosidl_default_runtime)

###########
## Build ##
###########
include_directories(include)
#add_executable(target_recognition_service src/services/target_recognition_service.cpp)
#add_dependencies(target_recognition_service ${PROJECT_NAME}_generate_messages_cpp)
#target_link_libraries(target_recognition_service ${catkin_LIBRARIES})

#add_executable(generate_cloud_node src/nodes/generate_point_cloud.cpp)
#add_dependencies(generate_cloud_node ${PROJECT_NAME}_generate_messages_cpp)
#target_link_libraries(generate_cloud_node ${catkin_LIBRARIES})

add_executable(pick_and_place_node 
	src/nodes/pick_and_place_node.cpp
  src/tasks/initialize.cpp
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
ament_target_dependencies(pick_and_place_node
  pick_and_place_msgs
  moveit_common
  moveit_ros_planning_interface)

#############
## Install ##
#############

install(TARGETS
  pick_and_place_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

ament_package()
