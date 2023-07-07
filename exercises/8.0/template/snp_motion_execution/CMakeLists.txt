cmake_minimum_required(VERSION 3.5)
project(snp_motion_execution)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(control_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(snp_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

add_executable(${PROJECT_NAME}_node src/motion_execution_node.cpp)
ament_target_dependencies(
  ${PROJECT_NAME}_node
  control_msgs
  rclcpp
  rclcpp_action
  sensor_msgs
  snp_msgs
  std_srvs)

add_executable(execution_simulator src/execution_simulator_node.cpp)
ament_target_dependencies(
  execution_simulator
  control_msgs
  rclcpp
  rclcpp_action)

add_executable(robot_enable_simulator src/robot_enable_simulator_node.cpp)
ament_target_dependencies(robot_enable_simulator rclcpp std_srvs)

install(TARGETS ${PROJECT_NAME}_node execution_simulator robot_enable_simulator DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_dependencies(
  control_msgs
  rclcpp
  rclcpp_action
  snp_msgs
  std_srvs
  sensor_msgs)
ament_package()
