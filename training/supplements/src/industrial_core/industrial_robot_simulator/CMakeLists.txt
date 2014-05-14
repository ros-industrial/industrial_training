cmake_minimum_required(VERSION 2.8.3)

project(industrial_robot_simulator)

find_package(catkin REQUIRED COMPONENTS roscpp std_msgs sensor_msgs 
  control_msgs trajectory_msgs industrial_robot_client)

catkin_package(
    CATKIN_DEPENDS roscpp std_msgs sensor_msgs control_msgs trajectory_msgs industrial_robot_client
)

##########
## Test ##
##########

# ROS launch testing
## ROS launch testing should be re-enabled when the roslaunch script is officially
## released (see https://github.com/ros/ros_comm/pull/333)
##find_package(roslaunch)
##roslaunch_add_file_check(launch)

#############
## Install ##
#############


install(PROGRAMS industrial_robot_simulator DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(DIRECTORY launch/ DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch)
