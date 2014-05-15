cmake_minimum_required(VERSION 2.8.3)
project(abb_moveit_plugins)
find_package(catkin REQUIRED COMPONENTS moveit_core roscpp)

#######################################
## Adding directories and definitions #
#######################################
include_directories(include ${Boost_INCLUDE_DIR} ${catkin_INCLUDE_DIRS})

###################################
## catkin specific configuration ##
###################################
## Declare artifacts to be passed to dependent projects
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
    DEPENDS 
    CATKIN_DEPENDS moveit_core roscpp
    INCLUDE_DIRS include
    LIBRARIES 
)

###########
## Build ##
###########

# Library containing IKFast plugin
add_library(irb_2400_moveit_arm_kinematics
  src/irb_2400_manipulator_ikfast_moveit_plugin.cpp
  src/irb_2400_manipulator_ikfast_solver.cpp)

#############
## Install ##
#############

install(
  TARGETS irb_2400_moveit_arm_kinematics
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})

install(
  FILES irb_2400_moveit_kinematics_plugin.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
