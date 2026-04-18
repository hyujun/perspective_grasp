#include <gtest/gtest.h>

#include <cmath>
#include <limits>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <sensor_msgs/msg/region_of_interest.hpp>

#include "test_helpers.hpp"
#include "yolo_pcl_cpp_tracker/pcl_utils.hpp"

namespace perspective_grasp {
namespace {

using test_helpers::makeCubeCloud;
using test_helpers::makeOrganizedCloud;

sensor_msgs::msg::RegionOfInterest makeRoi(uint32_t x, uint32_t y, uint32_t w,
                                           uint32_t h) {
  sensor_msgs::msg::RegionOfInterest roi;
  roi.x_offset = x;
  roi.y_offset = y;
  roi.width = w;
  roi.height = h;
  return roi;
}

// ---------------- cropToRoi ----------------

TEST(CropToRoi, OrganizedCloudExactCount) {
  auto cloud = makeOrganizedCloud(10, 10);
  auto roi = makeRoi(2, 3, 4, 5);  // 4 * 5 = 20 valid points expected

  auto cropped = pcl_utils::cropToRoi(*cloud, roi);
  ASSERT_NE(cropped, nullptr);
  EXPECT_EQ(cropped->size(), 20u);
  EXPECT_EQ(cropped->height, 1u);
  EXPECT_TRUE(cropped->is_dense);
}

TEST(CropToRoi, ClampsRoiToCloudBounds) {
  auto cloud = makeOrganizedCloud(8, 6);
  // ROI extends beyond the cloud — should be clamped, not crash
  auto roi = makeRoi(5, 4, 100, 100);

  auto cropped = pcl_utils::cropToRoi(*cloud, roi);
  ASSERT_NE(cropped, nullptr);
  // x in [5,8), y in [4,6) = 3 * 2 = 6 points
  EXPECT_EQ(cropped->size(), 6u);
}

TEST(CropToRoi, SkipsNaNAndNonPositiveDepth) {
  auto cloud = makeOrganizedCloud(4, 4, 1.0f, 0.0f);
  // Inject NaN at (1,1), z=0 at (2,1), z=-1 at (2,2)
  cloud->at(1, 1).z = std::numeric_limits<float>::quiet_NaN();
  cloud->at(2, 1).z = 0.0f;
  cloud->at(2, 2).z = -1.0f;

  auto roi = makeRoi(0, 0, 4, 4);  // full cloud = 16, expect 13 kept
  auto cropped = pcl_utils::cropToRoi(*cloud, roi);
  EXPECT_EQ(cropped->size(), 13u);
}

TEST(CropToRoi, UnorganizedCloudReturnsEmpty) {
  pcl::PointCloud<pcl::PointXYZ> unorganized;
  unorganized.emplace_back(0.0f, 0.0f, 1.0f);
  unorganized.emplace_back(0.1f, 0.1f, 1.0f);
  unorganized.width = 2;
  unorganized.height = 1;  // height=1 => unorganized
  unorganized.is_dense = true;

  auto cropped = pcl_utils::cropToRoi(unorganized, makeRoi(0, 0, 2, 1));
  ASSERT_NE(cropped, nullptr);
  EXPECT_EQ(cropped->size(), 0u);
}

TEST(CropToRoi, ZeroSizedRoiReturnsEmpty) {
  auto cloud = makeOrganizedCloud(5, 5);
  auto cropped = pcl_utils::cropToRoi(*cloud, makeRoi(1, 1, 0, 0));
  EXPECT_EQ(cropped->size(), 0u);
}

// ---------------- preprocess ----------------

TEST(Preprocess, VoxelDownsampleReducesCount) {
  auto cloud = makeCubeCloud(0.1, 10);  // ~600 points
  const size_t n_before = cloud->size();

  auto out = pcl_utils::preprocess(cloud,
                                   /*voxel=*/0.02, /*mean_k=*/10,
                                   /*stddev=*/3.0);
  ASSERT_NE(out, nullptr);
  EXPECT_GT(out->size(), 0u);
  EXPECT_LT(out->size(), n_before)
      << "voxel downsampling should reduce point count";
}

TEST(Preprocess, OutlierRemovalDropsInjectedOutliers) {
  auto cloud = makeCubeCloud(0.1, 10);
  // Inject a handful of outliers ~50cm from the cluster
  for (int i = 0; i < 5; ++i) {
    cloud->emplace_back(0.5f + static_cast<float>(i) * 0.05f, 0.5f, 0.5f);
  }
  const size_t n_with_outliers = cloud->size();

  auto out = pcl_utils::preprocess(cloud,
                                   /*voxel=*/0.002, /*mean_k=*/20,
                                   /*stddev=*/1.0);
  ASSERT_NE(out, nullptr);
  EXPECT_LT(out->size(), n_with_outliers);

  // No remaining point should be near the outlier coordinates
  for (const auto& pt : *out) {
    EXPECT_LT(std::abs(pt.x), 0.2f);
    EXPECT_LT(std::abs(pt.y), 0.2f);
    EXPECT_LT(std::abs(pt.z), 0.2f);
  }
}

TEST(Preprocess, EmptyCloudAfterVoxelReturnsEmpty) {
  auto cloud = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
  auto out = pcl_utils::preprocess(cloud, 0.01, 10, 1.0);
  ASSERT_NE(out, nullptr);
  EXPECT_EQ(out->size(), 0u);
}

}  // namespace
}  // namespace perspective_grasp
