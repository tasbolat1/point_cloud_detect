cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

project(region_growing_rgb_segmentation)
set (CMAKE_CXX_STANDARD 14)


find_package(PCL 1.9 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

set(DEPRECATION_FLAG "-Wno-deprecated-declarations")

#add_executable (extract_indices src/extract_indices.cpp)
#target_link_libraries (extract_indices ${PCL_LIBRARIES})


#add_executable (region_growing_rgb_segmentation src/region_growing_rgb_segmentation.cpp)
#target_link_libraries (region_growing_rgb_segmentation ${PCL_LIBRARIES})

add_executable (region_clustering src/region_clustering.cpp)
target_link_libraries (region_clustering ${PCL_LIBRARIES})

add_executable (view src/view.cpp)
target_link_libraries (view ${PCL_LIBRARIES})

