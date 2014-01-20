cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

project(cloudviewer)

find_package(PCL 1.2 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable (cloudviewer cloudviewer.cpp)
target_link_libraries (cloudviewer ${PCL_LIBRARIES} vtkHybrid vtkRendering pcl_apps_release pcl_common_release pcl_features_release pcl_filters_release pcl_io_ply_release pcl_io_release pcl_kdtree_release pcl_keypoints_release pcl_octree_release pcl_registration_release pcl_sample_consensus_release pcl_search_release pcl_segmentation_release pcl_surface_release pcl_tracking_release pcl_visualization_release)