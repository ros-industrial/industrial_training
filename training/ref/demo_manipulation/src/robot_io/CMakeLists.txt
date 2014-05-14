cmake_minimum_required(VERSION 2.8.3)
project(robot_io)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS 
  roscpp 
  object_manipulation_msgs
  actionlib
	ur_msgs
  message_generation  
)


#######################################
## Declare ROS messages and services ##
#######################################
## Generate services in the 'srv' folder
add_service_files(
   FILES
   DigitalOutputUpdate.srv
)

## Generate added messages and services with any dependencies listed here
 generate_messages()

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
	CATKIN_DEPENDS 
		roscpp
		message_runtime
		ur_msgs
		actionlib_msgs
		actionlib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(include
  ${catkin_INCLUDE_DIRS}
)

#add_executable(simple_grasp_action_server src/nodes/open_close_grasp_action_server.cpp)
#target_link_libraries(simple_grasp_action_server ${catkin_LIBRARIES})

add_executable(suction_gripper_action_server src/nodes/suction_gripper_action_server.cpp)
target_link_libraries(suction_gripper_action_server ${catkin_LIBRARIES})

#add_executable(digital_output_server src/nodes/digital_output_update_server.cpp)

add_executable(simulated_grasp_action_server src/nodes/simulated_grasp_action_server.cpp)
target_link_libraries(simulated_grasp_action_server ${catkin_LIBRARIES})


