cmake_minimum_required(VERSION 2.8.3)
project(lesson_move_group)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS 
  roscpp 
  message_generation
)

#uncomment if you have defined messages
#rosbuild_genmsg()
#uncomment if you have defined services
#rosbuild_gensrv()

rosbuild_add_executable(lesson_move_group_1 src/lesson_move_group_1.cpp)
target_link_libraries(lesson_move_group_1 moveit_move_group_interface)

rosbuild_add_executable(lesson_move_group_2 src/lesson_move_group_2.cpp)
target_link_libraries(lesson_move_group_2 moveit_move_group_interface)

rosbuild_add_executable(lesson_move_group_3 src/lesson_move_group_3.cpp)
target_link_libraries(lesson_move_group_3 moveit_move_group_interface)

rosbuild_add_executable(lesson_move_group_4 src/lesson_move_group_4.cpp)
target_link_libraries(lesson_move_group_4 moveit_move_group_interface)

rosbuild_add_executable(lesson_move_group_5 src/lesson_move_group_5.cpp)
target_link_libraries(lesson_move_group_5 moveit_move_group_interface)

#common commands for building c++ executables and libraries
#rosbuild_add_library(${PROJECT_NAME} src/example.cpp)
#target_link_libraries(${PROJECT_NAME} another_library)
#rosbuild_add_boost_directories()
#rosbuild_link_boost(${PROJECT_NAME} thread)
#rosbuild_add_executable(example examples/example.cpp)
#target_link_libraries(example ${PROJECT_NAME})
