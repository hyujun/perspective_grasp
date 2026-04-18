cmake_minimum_required(VERSION 3.20)
project(pose_graph_smoother VERSION 0.1.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
add_compile_options(-Wall -Wextra -Wpedantic -Wshadow -Wconversion)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(perception_msgs REQUIRED)
find_package(Eigen3 REQUIRED)

# Optional GTSAM support
find_package(GTSAM QUIET)
if(GTSAM_FOUND)
  message(STATUS "GTSAM found - building with full smoother support")
  add_definitions(-DHAS_GTSAM)
else()
  message(STATUS "GTSAM not found - building passthrough stub")
endif()

# Node class as a library so smoke tests can link it without main()
add_library(smoother_node_core src/smoother_node.cpp)
target_include_directories(smoother_node_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(smoother_node_core
  rclcpp
  rclcpp_lifecycle
  geometry_msgs
  tf2_ros
  tf2_eigen
  perception_msgs
)
target_link_libraries(smoother_node_core Eigen3::Eigen)
if(GTSAM_FOUND)
  target_link_libraries(smoother_node_core gtsam)
endif()

# Executable is a thin wrapper around the node library
add_executable(smoother_node src/smoother_main.cpp)
target_link_libraries(smoother_node smoother_node_core)
ament_target_dependencies(smoother_node rclcpp rclcpp_lifecycle)

install(TARGETS smoother_node DESTINATION lib/${PROJECT_NAME})
install(TARGETS smoother_node_core
  EXPORT ${PROJECT_NAME}_targets
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  INCLUDES DESTINATION include)
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME})

ament_export_targets(${PROJECT_NAME}_targets HAS_LIBRARY_TARGET)
ament_export_dependencies(rclcpp rclcpp_lifecycle perception_msgs
  geometry_msgs tf2_ros tf2_eigen)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(lifecycle_msgs REQUIRED)

  ament_add_gtest(test_smoother_node_smoke test/test_smoother_node_smoke.cpp)
  target_link_libraries(test_smoother_node_smoke smoother_node_core)
  ament_target_dependencies(test_smoother_node_smoke lifecycle_msgs)
endif()

ament_package()
