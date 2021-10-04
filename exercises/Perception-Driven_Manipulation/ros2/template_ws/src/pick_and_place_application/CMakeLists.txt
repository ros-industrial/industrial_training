cmake_minimum_required(VERSION 3.5)
project(pick_and_place_application)

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
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(shape_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(pick_and_place_msgs REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(moveit_common REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(yaml-cpp REQUIRED)

find_package(Boost REQUIRED COMPONENTS system)

###########
## Build ##
###########
include_directories(include)

add_executable(generate_cloud_node src/nodes/generate_point_cloud.cpp)
ament_target_dependencies(generate_cloud_node rclcpp sensor_msgs tf2 tf2_ros pcl_ros)
target_link_libraries(generate_cloud_node yaml-cpp)

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
  
add_executable(fake_grasp_server_node
  src/nodes/fake_grasp_action_server.cpp)
ament_target_dependencies(fake_grasp_server_node
  rclcpp
  rclcpp_action
  pick_and_place_msgs)

#############
## Install ##
#############

install(TARGETS 
  generate_cloud_node
  pick_and_place_node
  fake_grasp_server_node              
  DESTINATION lib/${PROJECT_NAME})
  
install(PROGRAMS
  src/nodes/fake_recognition_service.py
  DESTINATION lib/${PROJECT_NAME}
)
  
install(DIRECTORY
 launch
 config
 DESTINATION share/${PROJECT_NAME}/
)

ament_package()
