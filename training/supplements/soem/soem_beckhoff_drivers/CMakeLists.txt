cmake_minimum_required(VERSION 2.6.3)

project(soem_beckhoff_drivers)
# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
#set(ROS_BUILD_TYPE RelWithDebInfo)

 
# Do setup in case of ros package, If ROS_ROOT is set, it is                                                                                                 # recommended to use RTT/OCL through the ros packages.                                                                                                       
set (ROS_ROOT $ENV{ROS_ROOT} )
if (ROS_ROOT)
  include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)
  rosbuild_init()
  rosbuild_find_ros_package( rtt )
  set( RTT_HINTS HINTS ${rtt_PACKAGE_PATH}/../install )
endif()

#uncomment if you have defined messages
rosbuild_genmsg()

# Set the CMAKE_PREFIX_PATH in case you're not using Orocos through
# ROS for helping these find commands find RTT.
find_package(OROCOS-RTT REQUIRED ${RTT_HINTS} )

# Defines the orocos_* cmake macros. See that file for additional
# documentation.
include(${OROCOS-RTT_USE_FILE_PATH}/UseOROCOS-RTT.cmake)

#rosbuild_include(rtt_rosnode GenerateRTTtypekit)

#ros_generate_rtt_typekit(soem_beckhoff_drivers)

orocos_plugin(soem_el1xxx src/soem_beckhoff_drivers.cpp src/soem_el1xxx.cpp)
orocos_plugin(soem_el2xxx src/soem_beckhoff_drivers.cpp src/soem_el2xxx.cpp)
orocos_plugin(soem_el4xxx src/soem_beckhoff_drivers.cpp src/soem_el4xxx.cpp)
orocos_plugin(soem_el3102 src/soem_beckhoff_drivers.cpp src/soem_el3102.cpp)
orocos_plugin(soem_el5101 src/soem_beckhoff_drivers.cpp src/soem_el5101.cpp)
orocos_plugin(soem_el6022 src/soem_beckhoff_drivers.cpp src/soem_el6022.cpp)
orocos_plugin(soem_el30xx src/soem_beckhoff_drivers.cpp src/soem_el30xx.cpp)
orocos_plugin(soem_el3104 src/soem_beckhoff_drivers.cpp src/soem_el3104.cpp)



