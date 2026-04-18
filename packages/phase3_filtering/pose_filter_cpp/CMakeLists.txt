cmake_minimum_required(VERSION 3.20)
project(pose_filter_cpp VERSION 0.1.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
add_compile_options(-Wall -Wextra -Wpedantic)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(perception_msgs REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(Eigen3 REQUIRED)

# IEKF algorithm library (reusable without ROS)
add_library(iekf_se3_lib src/iekf_se3.cpp)
target_include_directories(iekf_se3_lib PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(iekf_se3_lib PUBLIC Eigen3::Eigen)

# Node class as a library so smoke tests can link it without main()
add_library(pose_filter_node_core src/pose_filter_node.cpp)
target_include_directories(pose_filter_node_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(pose_filter_node_core
  rclcpp geometry_msgs tf2_ros tf2_eigen
  perception_msgs diagnostic_msgs)
target_link_libraries(pose_filter_node_core iekf_se3_lib)

# Executable is a thin wrapper around the node library
add_executable(pose_filter_node src/pose_filter_main.cpp)
target_link_libraries(pose_filter_node pose_filter_node_core)
ament_target_dependencies(pose_filter_node rclcpp)

install(TARGETS pose_filter_node DESTINATION lib/${PROJECT_NAME})
install(TARGETS iekf_se3_lib pose_filter_node_core
  EXPORT ${PROJECT_NAME}_targets
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  INCLUDES DESTINATION include)
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME})

ament_export_targets(${PROJECT_NAME}_targets HAS_LIBRARY_TARGET)
ament_export_dependencies(Eigen3 rclcpp perception_msgs diagnostic_msgs
  geometry_msgs tf2_ros tf2_eigen)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

  ament_add_gtest(test_iekf_se3 test/test_iekf_se3.cpp)
  target_link_libraries(test_iekf_se3 iekf_se3_lib)

  ament_add_gtest(test_pose_filter_node_smoke test/test_pose_filter_node_smoke.cpp)
  target_link_libraries(test_pose_filter_node_smoke pose_filter_node_core)
endif()

ament_package()
