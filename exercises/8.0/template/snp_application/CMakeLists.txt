cmake_minimum_required(VERSION 3.5)
project(snp_application)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)
set(CMAKE_AUTOUIC ON)

if(CMAKE_VERSION VERSION_LESS "3.7.0")
  set(CMAKE_INCLUDE_CURRENT_DIR ON)
endif()

find_package(Eigen3 REQUIRED)
find_package(Qt5 COMPONENTS Widgets REQUIRED)
find_package(yaml-cpp REQUIRED)

find_package(ament_cmake REQUIRED)
find_package(industrial_reconstruction_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(control_msgs REQUIRED)
find_package(snp_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(rviz_common REQUIRED)
find_package(pluginlib REQUIRED)

include_directories(include ${EIGEN3_INCLUDE_DIRS})

add_library(
  ${PROJECT_NAME}_widget SHARED
  src/snp_widget.ui
  include/std_msgs_yaml.h
  include/trajectory_msgs_yaml.h
  include/serialize.h
  include/snp_widget.h
  src/snp_widget.cpp)
target_link_libraries(${PROJECT_NAME}_widget Qt5::Widgets yaml-cpp)
ament_target_dependencies(
  ${PROJECT_NAME}_widget
  industrial_reconstruction_msgs
  rclcpp_action
  rclcpp_components
  control_msgs
  sensor_msgs
  geometry_msgs
  snp_msgs
  std_srvs
  tf2_eigen
  trajectory_msgs)

add_library(${PROJECT_NAME}_panel SHARED src/snp_panel.cpp)
target_link_libraries(${PROJECT_NAME}_panel ${PROJECT_NAME}_widget)
ament_target_dependencies(${PROJECT_NAME}_panel rviz_common pluginlib)

add_executable(roscon_app src/main.cpp)
target_link_libraries(roscon_app Qt5::Widgets ${PROJECT_NAME}_widget)
ament_target_dependencies(roscon_app rclcpp)

# Install the library(ies)
install(TARGETS ${PROJECT_NAME}_widget ${PROJECT_NAME}_panel EXPORT ${PROJECT_NAME}-targets DESTINATION lib)
ament_export_targets(${PROJECT_NAME}-targets HAS_LIBRARY_TARGET)

install(TARGETS roscon_app DESTINATION lib/${PROJECT_NAME}/)

pluginlib_export_plugin_description_file(rviz_common plugin_description.xml)
ament_package()
