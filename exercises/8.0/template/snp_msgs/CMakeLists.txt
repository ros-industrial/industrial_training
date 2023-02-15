cmake_minimum_required(VERSION 3.5)
project(snp_msgs)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(trajectory_msgs REQUIRED)

rosidl_generate_interfaces(
  ${PROJECT_NAME}
  "msg/ToolPath.msg"
  "msg/ToolPaths.msg"
  "srv/ExecuteMotionPlan.srv"
  "srv/GenerateMotionPlan.srv"
  "srv/GenerateToolPaths.srv"
  DEPENDENCIES
  geometry_msgs
  trajectory_msgs)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

# Ament package settings
ament_export_dependencies(geometry_msgs rosidl_default_runtime trajectory_msgs)
ament_package()
