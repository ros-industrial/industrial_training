cmake_minimum_required(VERSION 2.8.3)
project(myworkcell_support)

find_package(catkin REQUIRED COMPONENTS
  roscpp
)

################################################
## Declare ROS messages, services and actions ##
################################################


################################################
## Declare ROS dynamic reconfigure parameters ##
################################################


###################################
## catkin specific configuration ##
###################################
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES myworkcell_support
#  CATKIN_DEPENDS myworkcell_core roscpp
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS}
)


#############
## Install ##
#############


#############
## Testing ##
#############

