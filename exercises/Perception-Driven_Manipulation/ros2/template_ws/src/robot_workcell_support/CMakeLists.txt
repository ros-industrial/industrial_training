cmake_minimum_required(VERSION 3.5)
project(robot_workcell_support)

find_package(ament_cmake REQUIRED)


install(DIRECTORY
 meshes
 urdf
 rviz
 DESTINATION share/${PROJECT_NAME}/
)

ament_package()


