cmake_minimum_required(VERSION 3.5)
project(lesson_perception)
add_definitions(-std=c++14)

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

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(pcl_msgs REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(tf_conversions REQUIRED)
#find_package(PCL REQUIRED)

#include_directories(
#  ${catkin_INCLUDE_DIRS}
#  ${PCL_INCLUDE_DIRS}
#)

add_executable(perception_node src/perception_node.cpp)
ament_target_dependencies(perception_node rclcpp tf2_ros pcl_ros)

install(TARGETS
    perception_node
    DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
    launch
    DESTINATION share/${PROJECT_NAME}
)

#target_link_libraries(perception_node ${catkin_LIBRARIES} ${PCL_LIBRARIES})

ament_package()
