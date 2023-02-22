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

add_executable(${PROJECT_NAME}_simulator_node src/execution_simulator_node.cpp)
ament_target_dependencies(
  ${PROJECT_NAME}_simulator_node
  rclcpp
  std_srvs
  rclcpp_action
  control_msgs
  snp_msgs)

add_executable(${PROJECT_NAME}_node src/motion_execution_node.cpp)
ament_target_dependencies(
  ${PROJECT_NAME}_node
  rclcpp
  std_srvs
  rclcpp_action
  control_msgs
  snp_msgs
  sensor_msgs)

install(TARGETS ${PROJECT_NAME}_simulator_node ${PROJECT_NAME}_node DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
