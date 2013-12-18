cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

project(marker)

find_package (PCL 1.2 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable (marker marker.cpp)
add_executable (aruco_simple_board aruco_simple_board.cpp)

target_link_libraries (marker ${PCL_LIBRARIES})