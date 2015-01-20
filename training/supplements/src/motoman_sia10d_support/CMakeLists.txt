
cmake_minimum_required(VERSION 2.8.3)

project(motoman_sia10d_support)

find_package(catkin REQUIRED)

catkin_package()

find_package(roslaunch)
roslaunch_add_file_check(test/launch_test.xml)

foreach(dir config launch meshes urdf)
   install(DIRECTORY ${dir}/
      DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/${dir})
endforeach(dir)
