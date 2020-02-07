cmake_minimum_required(VERSION 3.5)
project(myworkcell_core)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(fake_ar_publisher REQUIRED)
find_package(geometry_msgs REQUIRED)

find_package(rosidl_default_generators REQUIRED)
rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/LocalizePart.srv"
  DEPENDENCIES geometry_msgs
)

## Declare a C++ executable
add_executable(myworkcell_node src/myworkcell_node.cpp)
ament_target_dependencies(myworkcell_node rclcpp fake_ar_publisher)
rosidl_target_interfaces(myworkcell_node ${PROJECT_NAME} "rosidl_typesupport_cpp")

add_executable(vision_node src/vision_node.cpp)
ament_target_dependencies(vision_node rclcpp fake_ar_publisher)
rosidl_target_interfaces(vision_node ${PROJECT_NAME} "rosidl_typesupport_cpp")

## Mark executables and/or libraries for installation
install(TARGETS myworkcell_node vision_node
    ARCHIVE DESTINATION lib/${PROJECT_NAME}
    LIBRARY DESTINATION lib/${PROJECT_NAME}
    RUNTIME DESTINATION lib/${PROJECT_NAME}
)

# ament_package is called after the targets
ament_export_dependencies(rclcpp fake_ar_publisher)
ament_package()
