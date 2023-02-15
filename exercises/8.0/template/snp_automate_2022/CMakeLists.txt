cmake_minimum_required(VERSION 3.5.0)

# Extract package name and version
find_package(ros_industrial_cmake_boilerplate REQUIRED)
extract_package_metadata(pkg)
project(${pkg_extracted_name} VERSION ${pkg_extracted_version} LANGUAGES CXX)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY launch
            config
            urdf
            meshes
  DESTINATION share/${PROJECT_NAME}/)

ament_package()
