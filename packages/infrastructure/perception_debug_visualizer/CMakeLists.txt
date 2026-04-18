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

# Overlay helpers — pure OpenCV drawing, testable without rclcpp
add_library(overlay src/overlay.cpp)
target_include_directories(overlay PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(overlay
  perception_msgs
)
target_link_libraries(overlay ${OpenCV_LIBRARIES})

# Node class as a library for smoke tests
add_library(visualizer_core src/visualizer_node.cpp)
target_include_directories(visualizer_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(visualizer_core
  rclcpp
  sensor_msgs
  geometry_msgs
  cv_bridge
  image_transport
  perception_msgs
)
target_link_libraries(visualizer_core overlay ${OpenCV_LIBRARIES})

add_executable(visualizer_node src/visualizer_main.cpp)
target_link_libraries(visualizer_node visualizer_core)
ament_target_dependencies(visualizer_node rclcpp)

install(TARGETS visualizer_node DESTINATION lib/${PROJECT_NAME})
install(TARGETS overlay visualizer_core
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

  ament_add_gtest(test_overlay test/test_overlay.cpp)
  target_link_libraries(test_overlay overlay ${OpenCV_LIBRARIES})

  ament_add_gtest(test_visualizer_smoke test/test_visualizer_smoke.cpp)
  target_link_libraries(test_visualizer_smoke visualizer_core)
endif()

ament_package()
