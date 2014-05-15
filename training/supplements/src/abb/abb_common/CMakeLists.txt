cmake_minimum_required(VERSION 2.8.3)
project(abb_common)
find_package(catkin REQUIRED COMPONENTS industrial_robot_client simple_message)


add_definitions(-DLINUXSOCKETS=1)  #build using LINUX SOCKETS libraries


catkin_package(
    CATKIN_DEPENDS industrial_robot_client simple_message
)

include_directories(
  include 
  ${Boost_INCLUDE_DIR} 
  ${catkin_INCLUDE_DIRS}
)


add_executable(abb_robot_state
  src/abb_robot_state_node.cpp
  src/abb_utils.cpp)
target_link_libraries(abb_robot_state
  industrial_robot_client
  simple_message
  ${catkin_LIBRARIES})
set_target_properties(
  abb_robot_state
  PROPERTIES OUTPUT_NAME robot_state PREFIX "")

add_executable(abb_motion_download_interface
  src/abb_joint_downloader_node.cpp
  src/abb_utils.cpp)
target_link_libraries(abb_motion_download_interface
  industrial_robot_client
  simple_message
  ${catkin_LIBRARIES})
set_target_properties(abb_motion_download_interface
  PROPERTIES OUTPUT_NAME motion_download_interface PREFIX "")


install(
  TARGETS
    abb_robot_state
    abb_motion_download_interface
  DESTINATION
    ${CATKIN_PACKAGE_BIN_DESTINATION}
)

foreach(dir launch meshes rapid urdf)
  install(
    DIRECTORY ${dir}/
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/${dir})
endforeach(dir)
