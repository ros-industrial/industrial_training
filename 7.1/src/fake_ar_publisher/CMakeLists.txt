cmake_minimum_required(VERSION 3.5)
project(fake_ar_publisher)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(fake_ar_publisher_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)

## Declare a C++ executable
add_executable(fake_ar_publisher_node src/fake_ar_publisher.cpp)

ament_target_dependencies(fake_ar_publisher_node rclcpp fake_ar_publisher_msgs visualization_msgs)

# add_dependencies(fake_ar_publisher_node ${PROJECT_NAME})
target_include_directories(fake_ar_publisher_node PUBLIC
                          ${rclcpp_INCLUDE_DIRS}
                          ${fake_ar_publisher_msgs_INCLUDE_DIRS}
                          ${std_msgs_INCLUDE_DIRS})

## Specify libraries to link a library or executable target against
target_link_libraries(fake_ar_publisher_node
                      ${rclcpp_LIBRARIES}
                      ${std_msgs_LIBRARIES})


# ament_package is called after the targets
ament_export_dependencies(rosidl_default_runtime)
ament_package()