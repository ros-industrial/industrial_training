cmake_minimum_required(VERSION 3.5)
project(myworkcell_msgs)

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
find_package(ament_cmake REQUIRED)

# find the ROS message code generators
find_package(builtin_interfaces REQUIRED)	
find_package(rosidl_default_generators REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)
#find_package(rosidl_default_generators REQUIRED)

# declare the service files to generate code for
set(srv_files
   "srv/LocalizePart.srv"
   "srv/PlanCartesianPath.srv"
   "srv/MoveToPose.srv"
   "srv/ExecuteTrajectory.srv"
 )

rosidl_generate_interfaces(${PROJECT_NAME}
   ${srv_files}
   "msg/ARMarker.msg"
   DEPENDENCIES geometry_msgs trajectory_msgs builtin_interfaces
)

#rosidl_generate_interfaces(new_msg
#  "msg/ARMarker.msg"
#  DEPENDENCIES builtin_interfaces
#)

ament_export_dependencies(rosidl_default_runtime)
ament_package()

install(
  FILES message_mappings.yaml
  DESTINATION share/${PROJECT_NAME})
