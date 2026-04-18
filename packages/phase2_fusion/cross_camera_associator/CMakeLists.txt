cmake_minimum_required(VERSION 3.20)
project(cross_camera_associator VERSION 0.1.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
add_compile_options(-Wall -Wextra -Wpedantic)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(perception_msgs REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(Eigen3 REQUIRED)

# Algorithm library (reusable without ROS)
add_library(cross_camera_lib
  src/hungarian_solver.cpp
  src/global_id_manager.cpp
  src/camera_pose_buffer.cpp)
target_include_directories(cross_camera_lib PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(cross_camera_lib rclcpp perception_msgs)
target_link_libraries(cross_camera_lib Eigen3::Eigen)

# ROS node
add_executable(associator_node src/associator_node.cpp)
target_include_directories(associator_node PRIVATE include)
ament_target_dependencies(associator_node
  rclcpp tf2_ros tf2_eigen tf2_geometry_msgs geometry_msgs perception_msgs diagnostic_msgs)
target_link_libraries(associator_node cross_camera_lib)

install(TARGETS associator_node DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_hungarian_solver test/test_hungarian_solver.cpp)
  target_link_libraries(test_hungarian_solver cross_camera_lib)
  ament_add_gtest(test_global_id_manager test/test_global_id_manager.cpp)
  target_link_libraries(test_global_id_manager cross_camera_lib)
endif()

ament_package()
