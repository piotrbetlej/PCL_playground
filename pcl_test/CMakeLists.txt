cmake_minimum_required(VERSION 2.8 FATAL_ERROR)
project(pcl_test)
find_package(PCL 1.8 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

set(PCL_BUILD_TYPE Release)

add_executable(pcl_test
# "./src/normals_main.cpp" "./src/main.cpp"
# "./src/GeneratedCloud.cpp" "./src/filecloud.cpp" "./src/GeneratedCloud.cpp"
# "./src/planar_range_image.cpp" "./src/range_image_visualization.cpp"
# "./src/borders.cpp" "./src/GeneratedRangeImage.cpp" "./src/euclid_clustering.cpp"
"./src/xtionviewer.cpp")

set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -Wall")
set(CMAKE_BUILD_TYPE Debug)

target_link_libraries (pcl_test ${PCL_LIBRARIES})
MESSAGE( STATUS "CMAKE_MODULE_PATH: " ${CMAKE_MODULE_PATH} )
