cmake_minimum_required(VERSION 2.8.3)
project(sensor_config)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS 
  roscpp 
  tf
  geometry_msgs
  roscpp
  visualization_msgs
)


###################################
## catkin specific configuration ##
###################################
catkin_package()



