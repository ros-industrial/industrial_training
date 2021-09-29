cmake_minimum_required(VERSION 3.8)
project(pick_and_place_msgs)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(action_msgs REQUIRED)
find_package(shape_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

set(service_files
  "srv/GetTargetPose.srv"
)

set(action_files
  "action/ExecuteGraspMove.action"
)

rosidl_generate_interfaces(${PROJECT_NAME}
  ${service_files}
  ${action_files}
  DEPENDENCIES
    action_msgs
    shape_msgs
    geometry_msgs
    builtin_interfaces
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
