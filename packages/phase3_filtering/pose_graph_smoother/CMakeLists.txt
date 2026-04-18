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

add_executable(smoother_node src/smoother_node.cpp)
target_include_directories(smoother_node PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/include
)
ament_target_dependencies(smoother_node
  rclcpp
  rclcpp_lifecycle
  geometry_msgs
  tf2_ros
  tf2_eigen
  perception_msgs
)
target_link_libraries(smoother_node Eigen3::Eigen)

if(GTSAM_FOUND)
  target_link_libraries(smoother_node gtsam)
endif()

install(TARGETS smoother_node DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
endif()

ament_package()
