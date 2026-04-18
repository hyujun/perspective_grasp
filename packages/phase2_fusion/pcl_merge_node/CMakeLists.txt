cmake_minimum_required(VERSION 3.20)
project(pcl_merge_node VERSION 0.1.0 LANGUAGES C CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
add_compile_options(-Wall -Wextra -Wpedantic)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_sensor_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(message_filters REQUIRED)
find_package(diagnostic_msgs REQUIRED)

# MPI before PCL to fix VTK dependency
find_package(MPI COMPONENTS C CXX QUIET)
find_package(PCL REQUIRED COMPONENTS common filters kdtree)

# Library
add_library(pcl_merge_lib
  src/cloud_preprocessor.cpp
  src/overlap_filter.cpp)
target_include_directories(pcl_merge_lib PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_link_libraries(pcl_merge_lib PUBLIC ${PCL_LIBRARIES})
target_include_directories(pcl_merge_lib PUBLIC ${PCL_INCLUDE_DIRS})

# Node class as a library so the smoke test can link it without main()
add_library(pcl_merge_node_core src/merge_node.cpp)
target_include_directories(pcl_merge_node_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(pcl_merge_node_core
  rclcpp sensor_msgs tf2_ros tf2_sensor_msgs
  pcl_conversions message_filters diagnostic_msgs)
target_link_libraries(pcl_merge_node_core pcl_merge_lib)

# Executable is a thin wrapper around the node library
add_executable(merge_node src/merge_main.cpp)
target_link_libraries(merge_node pcl_merge_node_core)
ament_target_dependencies(merge_node rclcpp)

install(TARGETS merge_node DESTINATION lib/${PROJECT_NAME})
install(TARGETS pcl_merge_lib pcl_merge_node_core
  EXPORT ${PROJECT_NAME}_targets
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

  ament_add_gtest(test_cloud_merger test/test_cloud_merger.cpp)
  target_link_libraries(test_cloud_merger pcl_merge_lib)

  ament_add_gtest(test_overlap_filter test/test_overlap_filter.cpp)
  target_link_libraries(test_overlap_filter pcl_merge_lib)

  ament_add_gtest(test_merge_node_smoke test/test_merge_node_smoke.cpp)
  target_link_libraries(test_merge_node_smoke pcl_merge_node_core)
endif()

ament_package()
