cmake_minimum_required(VERSION 2.8.3)
project(myworkcell_core)
add_definitions(-std=c++11)
find_package(catkin REQUIRED COMPONENTS
  roscpp
  ur5_demo_descartes
  descartes_trajectory
  descartes_moveit
  descartes_planner
  descartes_utilities
  message_generation
  std_msgs
  geometry_msgs
  tf
  moveit_ros_planning_interface
  roslint
  fake_ar_publisher
)

add_service_files(
  FILES
  LocalizePart.srv
  PlanCartesianPath.srv
)

generate_messages(
  DEPENDENCIES
  geometry_msgs
  std_msgs
  trajectory_msgs
)

catkin_package(
  CATKIN_DEPENDS roscpp message_runtime
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(vision_node src/vision_node.cpp)
add_executable(myworkcell_node src/myworkcell_node.cpp)
add_executable(descartes_node src/descartes_node.cpp)

add_dependencies(descartes_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(myworkcell_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(vision_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(vision_node ${catkin_LIBRARIES})
target_link_libraries(myworkcell_node ${catkin_LIBRARIES})
target_link_libraries(descartes_node ${catkin_LIBRARIES})

roslint_cpp()

if(CATKIN_ENABLE_TESTING)
    find_package(rostest REQUIRED)
    add_rostest_gtest(utest_node test/utest_launch.test src/test/utest.cpp)
    target_link_libraries(utest_node ${catkin_LIBRARIES})
endif()