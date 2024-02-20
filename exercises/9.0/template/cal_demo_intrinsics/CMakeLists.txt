cmake_minimum_required(VERSION 3.8)
project(cal_demo_intrinsics)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)

find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(camera_calibration_parsers REQUIRED)
find_package(ament_index_cpp REQUIRED)

add_executable(camera_info_publisher src/camera_info_publisher.cpp)
ament_target_dependencies(camera_info_publisher rclcpp sensor_msgs camera_calibration_parsers ament_index_cpp)

install(TARGETS camera_info_publisher
        DESTINATION lib/${PROJECT_NAME})


ament_package()
