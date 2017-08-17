cmake_minimum_required(VERSION 2.8.3)
project(ur5_demo_descartes)
add_compile_options(-std=c++11)
## Find catkin macros and libraries
find_package(catkin REQUIRED descartes_core descartes_moveit pluginlib moveit_core roscpp tf_conversions ur_kinematics)

find_package(rosconsole_bridge REQUIRED)
find_package(LAPACK REQUIRED)
find_package(Boost REQUIRED)
find_package(Eigen REQUIRED)

###################################
## catkin specific configuration ##
###################################
catkin_package(
  INCLUDE_DIRS
    include
  LIBRARIES
    ${PROJECT_NAME}
  CATKIN_DEPENDS
    moveit_core
    descartes_moveit
    pluginlib
    DEPENDS
)

###########
## Build ##
###########


include_directories(include
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  ${Eigen_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}
  src/ur5_robot_model.cpp
  src/ur_kin.cpp
  src/ur_moveit_plugin.cpp
  src/plugin_init.cpp)

target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES} ${Eigen_LIBRARIES}  ${Boost_LIBRARIES} ${LAPACK_LIBRARIES})

#############
## Install ##
#############


#############
## Testing ##
#############

