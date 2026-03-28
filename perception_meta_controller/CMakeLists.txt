cmake_minimum_required(VERSION 3.20)
project(perception_meta_controller VERSION 0.1.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
add_compile_options(-Wall -Wextra -Wpedantic -Wshadow -Wconversion)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(perception_msgs REQUIRED)

add_executable(meta_controller_node src/meta_controller_node.cpp)
target_include_directories(meta_controller_node PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/include
)
ament_target_dependencies(meta_controller_node
  rclcpp
  std_msgs
  perception_msgs
)

install(TARGETS meta_controller_node DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
endif()

ament_package()
