cmake_minimum_required(VERSION 2.8.3)
project(descartes_planner)

find_package(catkin REQUIRED COMPONENTS
  descartes_core
  descartes_trajectory
  moveit_core
  roscpp
  pluginlib
  cmake_modules
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
    descartes_planner
  CATKIN_DEPENDS
    roscpp
    moveit_core
    descartes_core
    descartes_trajectory
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

## DescartesTrajectoryPt lib
add_library(descartes_planner
            src/planning_graph.cpp
            src/sparse_planner.cpp
            src/dense_planner.cpp
            src/plugin_init.cpp
)

target_link_libraries(descartes_planner
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

  set(UTEST_SPARSE_PLANER_SRC_FILES test/utest.cpp
      test/sparse_planner.cpp)
  catkin_add_gtest(${PROJECT_NAME}_sparse_planner_utest ${UTEST_SPARSE_PLANER_SRC_FILES})
  target_link_libraries(${PROJECT_NAME}_sparse_planner_utest descartes_planner)


  set(UTEST_DENSE_PLANER_SRC_FILES test/utest.cpp
      test/dense_planner.cpp)
  catkin_add_gtest(${PROJECT_NAME}_dense_planner_utest ${UTEST_DENSE_PLANER_SRC_FILES})
  target_link_libraries(${PROJECT_NAME}_dense_planner_utest descartes_planner)

endif()
