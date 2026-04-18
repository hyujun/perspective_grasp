cmake_minimum_required(VERSION 3.20)
project(perception_debug_visualizer VERSION 0.1.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
add_compile_options(-Wall -Wextra -Wpedantic -Wshadow -Wconversion)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
find_package(perception_msgs REQUIRED)
find_package(OpenCV REQUIRED)

add_executable(visualizer_node src/visualizer_node.cpp)
target_include_directories(visualizer_node PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/include
)
ament_target_dependencies(visualizer_node
  rclcpp
  sensor_msgs
  geometry_msgs
  cv_bridge
  image_transport
  perception_msgs
)
target_link_libraries(visualizer_node ${OpenCV_LIBRARIES})

install(TARGETS visualizer_node DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
endif()

ament_package()
