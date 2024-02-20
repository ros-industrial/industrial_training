cmake_minimum_required(VERSION 3.8)
project(cal_demo_motion)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(tf2_eigen REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/MoveRobot.srv"
  DEPENDENCIES geometry_msgs trajectory_msgs
)

rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")

add_executable(motion_server src/motion_server.cpp)
ament_target_dependencies(motion_server rclcpp moveit_msgs moveit_ros_planning_interface tf2_eigen)
target_link_libraries(motion_server "${cpp_typesupport_target}")

install(TARGETS motion_server
    DESTINATION lib/${PROJECT_NAME})

install(PROGRAMS scripts/motion_client.py
        DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch config data
        DESTINATION share/${PROJECT_NAME})

ament_package()