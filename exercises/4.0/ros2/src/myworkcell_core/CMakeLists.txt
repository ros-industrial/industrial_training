cmake_minimum_required(VERSION 3.5)
project(myworkcell_core)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(fake_ar_publisher REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)

# Interface generation dependencies
find_package(geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/LocalizePart.srv"
  DEPENDENCIES geometry_msgs
)
rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")

# The vision_node executable
add_executable(vision_node src/vision_node.cpp)
ament_target_dependencies(vision_node rclcpp fake_ar_publisher tf2_ros tf2_geometry_msgs)
target_link_libraries(vision_node "${cpp_typesupport_target}")

# The myworkcell_node executable
add_executable(myworkcell_node src/myworkcell_node.cpp)
ament_target_dependencies(myworkcell_node rclcpp moveit_msgs moveit_ros_planning_interface)
target_link_libraries(myworkcell_node "${cpp_typesupport_target}")

# Mark executables for installation
install(TARGETS vision_node myworkcell_node
    DESTINATION lib/${PROJECT_NAME})

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
