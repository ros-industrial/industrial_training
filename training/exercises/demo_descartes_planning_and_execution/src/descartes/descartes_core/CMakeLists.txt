cmake_minimum_required(VERSION 2.8.3)
project(descartes_core)

find_package(catkin REQUIRED COMPONENTS
  moveit_core
  roscpp
  cmake_modules
)

find_package(Boost REQUIRED)
find_package(Eigen REQUIRED)

catkin_package(
  INCLUDE_DIRS
    include
  LIBRARIES
    descartes_core
  CATKIN_DEPENDS
    roscpp
    moveit_core
  DEPENDS
    Boost
    Eigen
  CFG_EXTRAS
    descartes_core-extras.cmake
)

###########
## Build ##
###########
include_directories(include
                    ${catkin_INCLUDE_DIRS}
                    ${Boost_INCLUDE_DIRS}
                    ${Eigen_INCLUDE_DIRS}
)

add_library(descartes_core
            src/trajectory_id.cpp
)

target_link_libraries(descartes_core
                      ${catkin_LIBRARIES}
)

#############
## Install ##
#############

install(
    TARGETS ${PROJECT_NAME} 
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})

install(
    DIRECTORY include/${PROJECT_NAME}/
    DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})
