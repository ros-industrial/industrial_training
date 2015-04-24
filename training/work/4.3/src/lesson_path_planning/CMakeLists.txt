cmake_minimum_required(VERSION 2.8.3)
project(lesson_path_planning)

find_package(catkin REQUIRED COMPONENTS
  descartes_core
  descartes_moveit
  descartes_trajectory
  descartes_planner
)

## System dependencies are found with CMake's conventions
find_package(Boost REQUIRED COMPONENTS system)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES plan_and_run
#  CATKIN_DEPENDS descartes_core descartes_planner trajectory_msgs
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
include_directories(
  ${catkin_INCLUDE_DIRS}
)

## Declare a cpp executable
add_executable(lesson_path_planning src/lesson_path_planning.cpp)
target_link_libraries(lesson_path_planning ${catkin_LIBRARIES})
