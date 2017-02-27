cmake_minimum_required(VERSION 2.8.3)
project(myworkcell_core)
add_definitions(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  fake_ar_publisher
  geometry_msgs
  message_generation
  tf
  moveit_ros_planning_interface
  ur5_demo_descartes
  descartes_trajectory
  descartes_planner
  descartes_utilities
  trajectory_msgs
)

add_service_files(
  FILES
  LocalizePart.srv
  PlanCartesianPath.srv
)

generate_messages(
  DEPENDENCIES
  geometry_msgs
  trajectory_msgs
)

catkin_package(
 CATKIN_DEPENDS
   roscpp
   fake_ar_publisher
   message_runtime
   geometry_msgs
   tf
   moveit_ros_planning_interface
   ur5_demo_descartes
   descartes_trajectory
   descartes_planner
   descartes_utilities
   trajectory_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(vision_node src/vision_node.cpp)
add_executable(myworkcell_node src/myworkcell_node.cpp)
add_executable(descartes_node src/descartes_node.cpp)

add_dependencies(vision_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(myworkcell_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(descartes_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(vision_node ${catkin_LIBRARIES})
target_link_libraries(myworkcell_node ${catkin_LIBRARIES})
target_link_libraries(descartes_node ${catkin_LIBRARIES})

if(CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)
  catkin_add_gtest(${PROJECT_NAME}_utest test/utest.cpp)
  target_link_libraries(${PROJECT_NAME}_utest ${catkin_LIBRARIES})
endif()
