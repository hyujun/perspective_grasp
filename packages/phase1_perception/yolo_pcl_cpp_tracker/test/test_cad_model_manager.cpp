#include <gtest/gtest.h>

#include <filesystem>
#include <fstream>
#include <string>

#include <pcl/io/pcd_io.h>

#include "yolo_pcl_cpp_tracker/cad_model_manager.hpp"

namespace perspective_grasp {
namespace {

/// Path to committed fixtures injected via `-D` (see CMakeLists.txt).
#ifndef YOLO_TRACKER_TEST_FIXTURES_DIR
#error "YOLO_TRACKER_TEST_FIXTURES_DIR must be defined by CMake"
#endif

TEST(CadModelManager, LoadsCommittedFixtures) {
  CadModelManager mgr;
  mgr.loadModels(YOLO_TRACKER_TEST_FIXTURES_DIR);

  EXPECT_TRUE(mgr.hasModels());
  EXPECT_GE(mgr.modelCount(), 2u)
      << "expected cube.pcd + sphere.pcd fixtures to load";

  auto cube = mgr.getModel("cube");
  ASSERT_NE(cube, nullptr);
  EXPECT_EQ(cube->size(), 8u);

  auto sphere = mgr.getModel("sphere");
  ASSERT_NE(sphere, nullptr);
  EXPECT_EQ(sphere->size(), 6u);
}

TEST(CadModelManager, UnknownClassReturnsNull) {
  CadModelManager mgr;
  mgr.loadModels(YOLO_TRACKER_TEST_FIXTURES_DIR);
  EXPECT_EQ(mgr.getModel("nonexistent"), nullptr);
}

TEST(CadModelManager, EmptyOnMissingDirectory) {
  CadModelManager mgr;
  mgr.loadModels("/tmp/definitely_does_not_exist_perspective_grasp_xyz");
  EXPECT_FALSE(mgr.hasModels());
  EXPECT_EQ(mgr.modelCount(), 0u);
}

TEST(CadModelManager, ReloadReplacesModels) {
  CadModelManager mgr;
  mgr.loadModels(YOLO_TRACKER_TEST_FIXTURES_DIR);
  const size_t first_count = mgr.modelCount();

  // Build a temp directory with a single .pcd (copy cube programmatically)
  auto tmp = std::filesystem::temp_directory_path() /
             "perspective_grasp_cad_reload_test";
  std::filesystem::create_directories(tmp);
  // Clear any leftovers from prior runs
  for (const auto& entry : std::filesystem::directory_iterator(tmp)) {
    std::filesystem::remove(entry.path());
  }

  auto cloud = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
  cloud->emplace_back(0.0f, 0.0f, 0.0f);
  cloud->emplace_back(1.0f, 0.0f, 0.0f);
  cloud->width = 2;
  cloud->height = 1;
  cloud->is_dense = true;
  ASSERT_EQ(pcl::io::savePCDFileASCII((tmp / "widget.pcd").string(), *cloud),
            0);

  mgr.loadModels(tmp.string());
  EXPECT_EQ(mgr.modelCount(), 1u);
  EXPECT_NE(mgr.getModel("widget"), nullptr);
  EXPECT_EQ(mgr.getModel("cube"), nullptr)
      << "reload should replace the prior model set, not merge";
  (void)first_count;

  std::filesystem::remove_all(tmp);
}

TEST(CadModelManager, IgnoresNonPcdFiles) {
  auto tmp = std::filesystem::temp_directory_path() /
             "perspective_grasp_cad_ignore_test";
  std::filesystem::create_directories(tmp);
  for (const auto& entry : std::filesystem::directory_iterator(tmp)) {
    std::filesystem::remove(entry.path());
  }

  // Non-PCD file — should be ignored
  std::ofstream(tmp / "README.txt") << "not a point cloud";

  // Valid PCD
  auto cloud = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
  cloud->emplace_back(0.0f, 0.0f, 0.0f);
  cloud->width = 1;
  cloud->height = 1;
  cloud->is_dense = true;
  ASSERT_EQ(pcl::io::savePCDFileASCII((tmp / "real.pcd").string(), *cloud), 0);

  CadModelManager mgr;
  mgr.loadModels(tmp.string());
  EXPECT_EQ(mgr.modelCount(), 1u);
  EXPECT_NE(mgr.getModel("real"), nullptr);

  std::filesystem::remove_all(tmp);
}

}  // namespace
}  // namespace perspective_grasp
