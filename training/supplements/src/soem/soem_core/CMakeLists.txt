cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
#set(ROS_BUILD_TYPE RelWithDebInfo)

rosbuild_init()

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

#uncomment if you have defined messages
#rosbuild_genmsg()
#uncomment if you have defined services
#rosbuild_gensrv()

#common commands for building c++ executables and libraries
find_path(RTNET_INCLUDE_DIR NAMES rtnet.h PATH_SUFFIXES rtnet/include)
option(ENABLE_RTNET "Enable RTnet over Xenomai." ON)
if(RTNET_INCLUDE_DIR AND ENABLE_RTNET)
  message("Found RTNET: ${RTNET_INCLUDE_DIR}")
  include_directories(${RTNET_INCLUDE_DIR})
  add_definitions(-DHAVE_RTNET)
  # try Xenomai
  execute_process(COMMAND xeno-config --skin=posix --cflags OUTPUT_VARIABLE XENO_CFLAGS OUTPUT_STRIP_TRAILING_WHITESPACE)
  execute_process(COMMAND xeno-config --skin=posix --ldflags OUTPUT_VARIABLE XENO_LDFLAGS OUTPUT_STRIP_TRAILING_WHITESPACE)
  set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${XENO_CFLAGS}")
  set(CMAKE_SHARED_LINKER_FLAGS "${CMAKE_SHARED_LINKER_FLAGS} ${XENO_LDFLAGS}")
  set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_SHARED_LINKER_FLAGS} ${XENO_LDFLAGS} -lnative")
endif()

rosbuild_add_library(${PROJECT_NAME} build/soem_core/src/ethercatbase.c build/soem_core/src/ethercatcoe.c build/soem_core/src/ethercatconfig.c build/soem_core/src/ethercatfoe.c build/soem_core/src/ethercatsoe.c build/soem_core/src/ethercatmain.c build/soem_core/src/ethercatprint.c build/soem_core/src/nicdrv.c build/soem_core/src/ethercatdc.c) 

include_directories(${PROJECT_SOURCE_DIR}/build/soem_core/src)
#rosbuild_add_boost_directories()
#rosbuild_link_boost(${PROJECT_NAME} thread)
rosbuild_add_executable(slaveinfo build/soem_core/src/slaveinfo.c)
target_link_libraries(slaveinfo ${PROJECT_NAME})
