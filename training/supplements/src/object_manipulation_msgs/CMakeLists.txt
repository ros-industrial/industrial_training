cmake_minimum_required(VERSION 2.8.3)
project(object_manipulation_msgs)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS 
	message_generation
	actionlib_msgs
	std_msgs
	geometry_msgs
	sensor_msgs
	trajectory_msgs
	sensor_msgs
	household_objects_database_msgs
	shape_msgs
	manipulation_msgs)

#######################################
## Declare ROS messages and services ##
#######################################

## Generate messages in the 'msg' folder
add_message_files(
   FILES
   ManipulationResult.msg
)

## Generate actions in the 'action' folder
add_action_files(
	DIRECTORY action
	FILES GraspHandPostureExecution.action
)

## Generate added messages and services with any dependencies listed here
generate_messages(DEPENDENCIES 	actionlib_msgs
	std_msgs
	geometry_msgs
	sensor_msgs
	trajectory_msgs
	sensor_msgs
	household_objects_database_msgs
	shape_msgs
	manipulation_msgs)

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
  CATKIN_DEPENDS 
		message_runtime 	
		actionlib_msgs
		std_msgs
		geometry_msgs
		sensor_msgs
		trajectory_msgs
		sensor_msgs
		household_objects_database_msgs
		shape_msgs
		manipulation_msgs
)
