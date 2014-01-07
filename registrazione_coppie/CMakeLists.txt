cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

project(registrazione_coppie)

find_package (PCL 1.2 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable (registrazione_coppie registrazione_coppie)

target_link_libraries (registrazione_coppie ${PCL_LIBRARIES} vtkHybrid vtkRendering)