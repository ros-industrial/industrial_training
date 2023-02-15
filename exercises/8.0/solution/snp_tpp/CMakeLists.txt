cmake_minimum_required(VERSION 3.5)
project(snp_tpp)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(Qt5 COMPONENTS Widgets REQUIRED)
find_package(noether_tpp REQUIRED)
find_package(noether_gui REQUIRED)
find_package(noether_filtering REQUIRED)
find_package(boost_plugin_loader REQUIRED)
find_package(PCL REQUIRED)

find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rviz_common REQUIRED)
find_package(rviz_polygon_selection_tool REQUIRED)
find_package(snp_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(tf2_eigen REQUIRED)

add_executable(${PROJECT_NAME}_node src/snp_tpp_node.cpp)
ament_target_dependencies(
  ${PROJECT_NAME}_node
  geometry_msgs
  rclcpp
  snp_msgs
  tf2_eigen)
target_link_libraries(${PROJECT_NAME}_node ${PCL_LIBRARIES} noether::noether_tpp)

add_executable(${PROJECT_NAME}_test src/snp_tpp_test_node.cpp)
ament_target_dependencies(
  ${PROJECT_NAME}_test
  geometry_msgs
  rclcpp
  snp_msgs)

# Custom mesh modifier
qt5_wrap_ui(${PROJECT_NAME}_ui_mocs src/roi_selection_mesh_modifier_widget.ui)
qt5_wrap_cpp(${PROJECT_NAME}_mocs src/roi_selection_mesh_modifier.h)

add_library(${PROJECT_NAME}_mesh_modifier SHARED src/roi_selection_mesh_modifier.cpp ${${PROJECT_NAME}_ui_mocs}
                                                 ${${PROJECT_NAME}_mocs})
target_include_directories(
  ${PROJECT_NAME}_mesh_modifier PUBLIC "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
                                       "$<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}>" "$<INSTALL_INTERFACE:include>")
target_link_libraries(
  ${PROJECT_NAME}_mesh_modifier
  noether::noether_tpp
  noether::noether_filtering
  noether::noether_gui)
ament_target_dependencies(
  ${PROJECT_NAME}_mesh_modifier
  geometry_msgs
  snp_msgs
  rviz_polygon_selection_tool
  rclcpp
  tf2_eigen
  std_srvs)

# Noether plugin library
add_library(${PROJECT_NAME}_plugins SHARED src/plugins.cpp)
target_link_libraries(${PROJECT_NAME}_plugins ${PROJECT_NAME}_mesh_modifier)

# Custom TPP widget
qt5_wrap_cpp(${PROJECT_NAME}_widget_mocs src/tpp_widget.h)
add_library(${PROJECT_NAME}_widget SHARED src/tpp_widget.cpp ${${PROJECT_NAME}_widget_mocs})
target_link_libraries(
  ${PROJECT_NAME}_widget
  noether::noether_gui
  boost_plugin_loader::boost_plugin_loader
  ${PCL_LIBRARIES})
ament_target_dependencies(${PROJECT_NAME}_widget snp_msgs tf2_eigen)
target_compile_definitions(${PROJECT_NAME}_widget PRIVATE "RVIZ_DEFAULT_PLUGINS_BUILDING_LIBRARY"
                                                          SNP_TPP_GUI_PLUGINS="${PROJECT_NAME}_plugins")

# TPP GUI application
add_executable(${PROJECT_NAME}_app src/tpp_app.cpp)
target_link_libraries(${PROJECT_NAME}_app ${PROJECT_NAME}_widget)
ament_target_dependencies(${PROJECT_NAME}_app rclcpp)

# Install the executables
install(TARGETS ${PROJECT_NAME}_node ${PROJECT_NAME}_test ${PROJECT_NAME}_app DESTINATION lib/${PROJECT_NAME})

# Install the library(ies)
install(TARGETS ${PROJECT_NAME}_mesh_modifier ${PROJECT_NAME}_plugins ${PROJECT_NAME}_widget
        EXPORT ${PROJECT_NAME}-targets DESTINATION lib)
ament_export_targets(${PROJECT_NAME}-targets HAS_LIBRARY_TARGET)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
