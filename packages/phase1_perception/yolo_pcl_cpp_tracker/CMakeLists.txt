cmake_minimum_required(VERSION 3.20)
project(yolo_pcl_cpp_tracker VERSION 0.1.0 LANGUAGES C CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
add_compile_options(-Wall -Wextra -Wpedantic)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(message_filters REQUIRED)
find_package(perception_msgs REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(teaser_icp_hybrid_registrator REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(MPI COMPONENTS C CXX QUIET)  # For PCL VTK dependency
find_package(PCL REQUIRED COMPONENTS common io filters registration search kdtree)
find_package(pcl_conversions REQUIRED)

# C++ ICP pose estimator node
add_executable(pcl_icp_pose_estimator
  src/pcl_icp_pose_estimator.cpp
  src/cad_model_manager.cpp
)
target_include_directories(pcl_icp_pose_estimator PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/include
)
target_include_directories(pcl_icp_pose_estimator SYSTEM PRIVATE
  ${PCL_INCLUDE_DIRS}
)
target_link_directories(pcl_icp_pose_estimator PRIVATE
  ${PCL_LIBRARY_DIRS}
)
ament_target_dependencies(pcl_icp_pose_estimator
  rclcpp
  sensor_msgs
  geometry_msgs
  tf2_ros
  tf2_eigen
  message_filters
  perception_msgs
  diagnostic_msgs
  pcl_conversions
  teaser_icp_hybrid_registrator
)
target_link_libraries(pcl_icp_pose_estimator
  ${PCL_LIBRARIES}
)

# Install C++ executable
install(TARGETS pcl_icp_pose_estimator
  DESTINATION lib/${PROJECT_NAME}
)

# Install Python scripts
install(PROGRAMS
  scripts/yolo_byte_tracker.py
  DESTINATION lib/${PROJECT_NAME}
)

# Install config and launch files
install(DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)

# Install resource marker for ament index
install(FILES resource/${PROJECT_NAME}
  DESTINATION share/ament_index/resource_index/packages
)

# Tests
if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  # Unit tests can be added here
endif()

ament_package()
