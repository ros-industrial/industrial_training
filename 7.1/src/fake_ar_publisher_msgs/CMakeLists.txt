cmake_minimum_required(VERSION 3.5)
project(fake_ar_publisher_msgs)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra)
endif()

find_package(ament_cmake)
find_package(geometry_msgs)
find_package(std_msgs)
find_package(builtin_interfaces REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(fake_ar_publisher_msgs
  "msg/ARMarker.msg"
  DEPENDENCIES builtin_interfaces geometry_msgs std_msgs
)

ament_package()