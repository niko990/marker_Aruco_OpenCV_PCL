cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

project(registrazione_multipla)

find_package (PCL 1.2 REQUIRED)
find_package (OPENCV  REQUIRED)
include_directories(${PCL_INCLUDE_DIRS} ${ARUKO_INCLUDE_DIR})
link_directories(${PCL_LIBRARY_DIRS} ${ARUKO_LIBRARY_DIR})
add_definitions(${PCL_DEFINITIONS})

add_executable (registrazione_multipla registrazione_multipla.cpp)

target_link_libraries (registrazione_multipla ${PCL_LIBRARIES} vtkHybrid vtkRendering ${OpenCV_LIBS} aruco124 )