cmake_minimum_required(VERSION 3.5)
project(ur5_workcell_moveit2_config)

find_package(ament_cmake REQUIRED)

install(DIRECTORY
 config
 DESTINATION share/${PROJECT_NAME}/
)

ament_package()
