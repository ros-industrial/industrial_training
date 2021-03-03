cmake_minimum_required(VERSION 3.5)
project(myworkcell_core)

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
find_package(fake_ar_publisher REQUIRED)

# Interface generation dependencies
find_package(geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/LocalizePart.srv"
  DEPENDENCIES geometry_msgs
)

# The vision_node executable
add_executable(vision_node src/vision_node.cpp)
ament_target_dependencies(vision_node rclcpp fake_ar_publisher)
rosidl_target_interfaces(vision_node ${PROJECT_NAME} "rosidl_typesupport_cpp")

# The myworkcell_node executable
add_executable(myworkcell_node src/myworkcell_node.cpp)
ament_target_dependencies(myworkcell_node rclcpp)
rosidl_target_interfaces(myworkcell_node ${PROJECT_NAME} "rosidl_typesupport_cpp")

# Mark executables and/or libraries for installation
install(TARGETS vision_node myworkcell_node
    ARCHIVE DESTINATION lib/${PROJECT_NAME}
    LIBRARY DESTINATION lib/${PROJECT_NAME}
    RUNTIME DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
