#
# descartes_core CMake/catkin extras snippet
#
# Enable (GCC) C++0x/11 support by adding the corresponding compiler flag
# directly.
#
# TODO: for newer versions of CMake, this should probably be done using
#       add_compile_options(..) (>= 2.8.11), or target_compile_features(..)
#       (>= 3.1.x).
#
if (NOT descartes_core_FIND_QUIETLY)
  message(STATUS "descartes_core: enabling C++0x (required)")
endif()

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
