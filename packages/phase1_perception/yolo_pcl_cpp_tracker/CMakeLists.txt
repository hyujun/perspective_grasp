cmake_minimum_required(VERSION 3.20)
project(yolo_pcl_cpp_tracker VERSION 0.1.0 LANGUAGES C CXX)

# Silence PCL's FindFLANN.cmake CMP0144 warning. Plain cmake_policy(SET) does not
# survive PCLConfig.cmake's own cmake_minimum_required, so use the *_DEFAULT_*
# variant which propagates into nested find_package calls.
if(POLICY CMP0144)
  set(CMAKE_POLICY_DEFAULT_CMP0144 NEW)
endif()

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
add_compile_options(-Wall -Wextra -Wpedantic)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(message_filters REQUIRED)
find_package(perception_msgs REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(teaser_icp_hybrid_registrator REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(MPI COMPONENTS C CXX QUIET)  # For PCL VTK dependency
find_package(PCL REQUIRED COMPONENTS common io filters registration search kdtree)
find_package(pcl_conversions REQUIRED)

# Shared static library of pure-logic utilities (testable without ROS init)
add_library(yolo_tracker_utils STATIC
  src/cad_model_manager.cpp
  src/pcl_utils.cpp
)
target_include_directories(yolo_tracker_utils PUBLIC
  ${CMAKE_CURRENT_SOURCE_DIR}/include
)
target_include_directories(yolo_tracker_utils SYSTEM PUBLIC
  ${PCL_INCLUDE_DIRS}
)
target_link_directories(yolo_tracker_utils PUBLIC
  ${PCL_LIBRARY_DIRS}
)
ament_target_dependencies(yolo_tracker_utils
  sensor_msgs
)
target_link_libraries(yolo_tracker_utils
  Eigen3::Eigen
  ${PCL_LIBRARIES}
)
set_target_properties(yolo_tracker_utils PROPERTIES POSITION_INDEPENDENT_CODE ON)

# C++ ICP pose estimator node
add_executable(pcl_icp_pose_estimator
  src/pcl_icp_pose_estimator.cpp
)
target_include_directories(pcl_icp_pose_estimator PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/include
)
target_include_directories(pcl_icp_pose_estimator SYSTEM PRIVATE
  ${PCL_INCLUDE_DIRS}
)
target_link_directories(pcl_icp_pose_estimator PRIVATE
  ${PCL_LIBRARY_DIRS}
)
ament_target_dependencies(pcl_icp_pose_estimator
  rclcpp
  sensor_msgs
  geometry_msgs
  tf2_ros
  tf2_eigen
  message_filters
  perception_msgs
  diagnostic_msgs
  pcl_conversions
  teaser_icp_hybrid_registrator
)
target_link_libraries(pcl_icp_pose_estimator
  yolo_tracker_utils
  ${PCL_LIBRARIES}
)

# Install C++ executable
install(TARGETS pcl_icp_pose_estimator
  DESTINATION lib/${PROJECT_NAME}
)

# Install Python scripts
install(PROGRAMS
  scripts/yolo_byte_tracker.py
  DESTINATION lib/${PROJECT_NAME}
)

# Install config and launch files
install(DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)

# Install resource marker for ament index
install(FILES resource/${PROJECT_NAME}
  DESTINATION share/ament_index/resource_index/packages
)

# Tests
if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

  # ObjectTracker — header-only struct, no link deps needed beyond Eigen/PCL.
  ament_add_gtest(test_object_tracker test/test_object_tracker.cpp)
  target_include_directories(test_object_tracker PRIVATE include test)
  target_link_libraries(test_object_tracker yolo_tracker_utils)

  # CadModelManager — needs the PCD fixtures dir injected as a define.
  ament_add_gtest(test_cad_model_manager test/test_cad_model_manager.cpp)
  target_include_directories(test_cad_model_manager PRIVATE include test)
  target_compile_definitions(test_cad_model_manager PRIVATE
    YOLO_TRACKER_TEST_FIXTURES_DIR=\"${CMAKE_CURRENT_SOURCE_DIR}/test/fixtures\"
  )
  target_link_libraries(test_cad_model_manager yolo_tracker_utils)

  # pcl_utils — cropToRoi / preprocess
  ament_add_gtest(test_pcl_utils test/test_pcl_utils.cpp)
  target_include_directories(test_pcl_utils PRIVATE include test)
  ament_target_dependencies(test_pcl_utils sensor_msgs)
  target_link_libraries(test_pcl_utils yolo_tracker_utils)
endif()

ament_package()
