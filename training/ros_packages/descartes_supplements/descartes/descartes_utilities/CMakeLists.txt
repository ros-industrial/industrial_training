cmake_minimum_required(VERSION 2.8.3)
project(descartes_utilities)

find_package(catkin REQUIRED COMPONENTS
  descartes_core
  descartes_trajectory
  trajectory_msgs
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES descartes_utilities
  CATKIN_DEPENDS descartes_core descartes_trajectory trajectory_msgs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}
  src/ros_conversions.cpp
)

add_dependencies(${PROJECT_NAME} 
  ${catkin_EXPORTED_TARGETS}
)

target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
)

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
)
