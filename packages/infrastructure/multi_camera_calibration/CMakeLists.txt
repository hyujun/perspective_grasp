cmake_minimum_required(VERSION 3.20)
project(multi_camera_calibration VERSION 0.1.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
add_compile_options(-Wall -Wextra -Wpedantic)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(std_srvs REQUIRED)

# Ceres is optional - enables joint optimization when available
find_package(Ceres QUIET)
if(Ceres_FOUND)
  message(STATUS "Ceres Solver found - joint optimization enabled")
else()
  message(WARNING "Ceres Solver not found - joint optimization disabled. "
    "Install libceres-dev for multi-camera joint optimization.")
endif()

# Calibration library
add_library(calibration_lib
  src/charuco_detector.cpp
  src/hand_eye_solver.cpp
  src/joint_optimizer.cpp
)
target_include_directories(calibration_lib PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(calibration_lib
  rclcpp
)
target_link_libraries(calibration_lib
  Eigen3::Eigen
  ${OpenCV_LIBS}
)

if(Ceres_FOUND)
  target_compile_definitions(calibration_lib PUBLIC HAS_CERES)
  target_link_libraries(calibration_lib Ceres::ceres)
endif()

# Offline calibration executable
add_executable(calibration_main src/calibration_main.cpp)
ament_target_dependencies(calibration_main
  rclcpp)
target_link_libraries(calibration_main calibration_lib)

# Install Python scripts
install(PROGRAMS
  scripts/collect_calibration_data.py
  scripts/generate_static_tf_launch.py
  DESTINATION lib/${PROJECT_NAME}
)

# Install targets
install(TARGETS calibration_main
  DESTINATION lib/${PROJECT_NAME}
)
install(TARGETS calibration_lib
  EXPORT calibration_lib_targets
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)

# Tests
if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

  ament_add_gtest(test_hand_eye_solver test/test_hand_eye_solver.cpp)
  target_link_libraries(test_hand_eye_solver calibration_lib)

  ament_add_gtest(test_pose_converters test/test_pose_converters.cpp)
  target_link_libraries(test_pose_converters calibration_lib)

  ament_add_gtest(test_charuco_detector test/test_charuco_detector.cpp)
  target_link_libraries(test_charuco_detector calibration_lib)

  ament_add_gtest(test_joint_optimizer test/test_joint_optimizer.cpp)
  target_link_libraries(test_joint_optimizer calibration_lib)
  if(Ceres_FOUND)
    target_compile_definitions(test_joint_optimizer PRIVATE HAS_CERES)
  endif()
endif()

ament_package()
