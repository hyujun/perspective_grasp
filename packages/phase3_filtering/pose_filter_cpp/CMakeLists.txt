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

# IEKF library (reusable without ROS)
add_library(iekf_se3_lib src/iekf_se3.cpp)
target_include_directories(iekf_se3_lib PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(iekf_se3_lib PUBLIC Eigen3::Eigen)

# ROS node
add_executable(pose_filter_node src/pose_filter_node.cpp)
target_include_directories(pose_filter_node PRIVATE include)
ament_target_dependencies(pose_filter_node
  rclcpp geometry_msgs tf2_ros tf2_eigen
  perception_msgs diagnostic_msgs)
target_link_libraries(pose_filter_node iekf_se3_lib)

install(TARGETS pose_filter_node DESTINATION lib/${PROJECT_NAME})
install(TARGETS iekf_se3_lib
  EXPORT iekf_se3_targets
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  INCLUDES DESTINATION include)
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME})

ament_export_targets(iekf_se3_targets HAS_LIBRARY_TARGET)
ament_export_dependencies(Eigen3)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_iekf_se3 test/test_iekf_se3.cpp)
  target_link_libraries(test_iekf_se3 iekf_se3_lib)
endif()

ament_package()
