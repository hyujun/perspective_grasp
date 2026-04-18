cmake_minimum_required(VERSION 3.20)
project(perception_meta_controller VERSION 0.1.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
add_compile_options(-Wall -Wextra -Wpedantic -Wshadow -Wconversion)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(perception_msgs REQUIRED)

# Pure logic library — testable without rclcpp init
add_library(mode_logic src/mode_logic.cpp)
target_include_directories(mode_logic PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(mode_logic
  rclcpp
  perception_msgs
)

# Node class as a library so the smoke test can link it without main()
add_library(meta_controller_core src/meta_controller_node.cpp)
target_include_directories(meta_controller_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(meta_controller_core
  rclcpp
  std_msgs
  perception_msgs
)
target_link_libraries(meta_controller_core mode_logic)

# Executable is a thin wrapper around the node library
add_executable(meta_controller_node src/meta_controller_main.cpp)
target_link_libraries(meta_controller_node meta_controller_core)
ament_target_dependencies(meta_controller_node rclcpp)

install(TARGETS meta_controller_node DESTINATION lib/${PROJECT_NAME})
install(TARGETS mode_logic meta_controller_core
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

  ament_add_gtest(test_mode_logic test/test_mode_logic.cpp)
  target_link_libraries(test_mode_logic mode_logic)

  ament_add_gtest(test_meta_controller_smoke test/test_meta_controller_smoke.cpp)
  target_link_libraries(test_meta_controller_smoke meta_controller_core)
endif()

ament_package()
