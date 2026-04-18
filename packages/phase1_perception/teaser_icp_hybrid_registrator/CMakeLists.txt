cmake_minimum_required(VERSION 3.20)
project(teaser_icp_hybrid_registrator VERSION 0.1.0 LANGUAGES C CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
add_compile_options(-Wall -Wextra -Wpedantic)

find_package(ament_cmake REQUIRED)
find_package(Eigen3 REQUIRED)
# PCL's VTK dependency requires MPI::MPI_C target to exist
find_package(MPI COMPONENTS C CXX QUIET)
find_package(PCL REQUIRED COMPONENTS common features filters registration search kdtree)

# TEASER++ is optional - enables global registration when available
find_package(teaserpp QUIET)
if(teaserpp_FOUND)
  message(STATUS "TEASER++ found - full hybrid registration enabled")
else()
  message(WARNING "TEASER++ not found - only ICP refinement available. "
    "Install TEASER++ for global registration: scripts/install_dependencies.sh")
endif()

# Library
add_library(teaser_icp_lib
  src/hybrid_registrator.cpp
  src/fpfh_feature_extractor.cpp
)
target_include_directories(teaser_icp_lib PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(teaser_icp_lib PUBLIC
  Eigen3::Eigen
  ${PCL_LIBRARIES}
)
target_include_directories(teaser_icp_lib SYSTEM PUBLIC
  ${PCL_INCLUDE_DIRS}
)
target_link_directories(teaser_icp_lib PUBLIC
  ${PCL_LIBRARY_DIRS}
)
target_compile_definitions(teaser_icp_lib PUBLIC
  ${PCL_DEFINITIONS}
)

if(teaserpp_FOUND)
  target_link_libraries(teaser_icp_lib PUBLIC teaserpp::teaser_registration)
  target_compile_definitions(teaser_icp_lib PUBLIC HAS_TEASERPP)
endif()

# Export for downstream packages
ament_export_targets(teaser_icp_targets HAS_LIBRARY_TARGET)
ament_export_dependencies(Eigen3 PCL)
if(teaserpp_FOUND)
  ament_export_dependencies(teaserpp)
endif()

install(TARGETS teaser_icp_lib
  EXPORT teaser_icp_targets
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)
install(DIRECTORY include/ DESTINATION include)

# Tests
if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

  ament_add_gtest(test_hybrid_registration test/test_hybrid_registration.cpp)
  target_include_directories(test_hybrid_registration PRIVATE test)
  target_link_libraries(test_hybrid_registration teaser_icp_lib)

  ament_add_gtest(test_fpfh_extractor test/test_fpfh_extractor.cpp)
  target_include_directories(test_fpfh_extractor PRIVATE test)
  target_link_libraries(test_fpfh_extractor teaser_icp_lib)
endif()

ament_package()
