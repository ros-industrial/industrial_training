cmake_minimum_required(VERSION 2.8.3)
project(descartes_trajectory)

find_package(catkin REQUIRED COMPONENTS
  cmake_modules
  descartes_core
  moveit_core
  roscpp
)

find_package(Boost REQUIRED)
find_package(Eigen REQUIRED)

###################################
## catkin specific configuration ##
###################################
catkin_package(
  INCLUDE_DIRS
    include
    test
  LIBRARIES
    descartes_trajectory
  CATKIN_DEPENDS
    roscpp
    moveit_core
    descartes_core
  DEPENDS
    Boost
    Eigen
)

###########
## Build ##
###########

include_directories(include
                    ${catkin_INCLUDE_DIRS}
                    ${Boost_INCLUDE_DIRS}
                    ${Eigen_INCLUDE_DIRS}
)

add_library(descartes_trajectory
            src/cart_trajectory_pt.cpp
            src/joint_trajectory_pt.cpp
            src/trajectory.cpp
            src/axial_symmetric_pt.cpp
            test/cartesian_robot.cpp
)

target_link_libraries(descartes_trajectory
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


#############
## Testing ##
#############

if(CATKIN_ENABLE_TESTING)
  set(UTEST_SRC_FILES test/utest.cpp
      test/trajectory_pt.cpp
      test/cart_trajectory_pt.cpp
      test/joint_trajectory_pt.cpp
      test/cartesian_robot.cpp
      test/cartesian_robot_test.cpp
      test/axial_symmetric_pt.cpp)
  catkin_add_gtest(${PROJECT_NAME}_utest ${UTEST_SRC_FILES})
  target_link_libraries(${PROJECT_NAME}_utest descartes_trajectory)

endif()
