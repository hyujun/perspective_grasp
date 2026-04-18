#include <gtest/gtest.h>

#include <pcl/filters/voxel_grid.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include "pcl_merge_node/cloud_preprocessor.hpp"

using perspective_grasp::CloudPreprocessor;
using perspective_grasp::PreprocessorParams;

namespace {

pcl::PointCloud<pcl::PointXYZ>::Ptr makeTestCloud(size_t n, float z_base) {
  auto cloud = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
  cloud->reserve(n);
  for (size_t i = 0; i < n; ++i) {
    float x = static_cast<float>(i % 100) * 0.001f;
    float y = static_cast<float>(i / 100) * 0.001f;
    float z = z_base + static_cast<float>(i % 10) * 0.01f;
    cloud->push_back(pcl::PointXYZ(x, y, z));
  }
  cloud->width = static_cast<uint32_t>(cloud->size());
  cloud->height = 1;
  cloud->is_dense = true;
  return cloud;
}

}  // namespace

// Test 1: Single cloud passthrough preprocessing reduces point count
// (StatisticalOutlierRemoval removes outliers, PassThrough filters Z range)
TEST(CloudMerger, SingleCloudPreprocessingReducesPoints) {
  PreprocessorParams params;
  params.table_height = 0.0;
  params.max_object_height = 0.30;
  params.outlier_mean_k = 10;
  params.outlier_stddev = 1.0;

  CloudPreprocessor preprocessor(params);

  // Create cloud with some points outside Z range
  auto cloud = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
  // Points within range [table_height - 0.05, table_height + max_object_height]
  // = [-0.05, 0.30]
  for (int i = 0; i < 500; ++i) {
    float x = static_cast<float>(i % 50) * 0.002f;
    float y = static_cast<float>(i / 50) * 0.002f;
    cloud->push_back(pcl::PointXYZ(x, y, 0.10f));  // within range
  }
  // Add points outside Z range
  for (int i = 0; i < 100; ++i) {
    cloud->push_back(pcl::PointXYZ(0.0f, 0.0f, 1.0f));  // above range
  }
  for (int i = 0; i < 100; ++i) {
    cloud->push_back(pcl::PointXYZ(0.0f, 0.0f, -0.5f));  // below range
  }
  cloud->width = static_cast<uint32_t>(cloud->size());
  cloud->height = 1;
  cloud->is_dense = true;

  auto result = preprocessor.process(cloud);

  // Should have fewer points (at least the 200 out-of-range removed)
  EXPECT_LT(result->size(), cloud->size());
  // Should have at most 500 points (the in-range ones)
  EXPECT_LE(result->size(), 500u);
  EXPECT_GT(result->size(), 0u);
}

// Test 2: Two identical clouds merged with VoxelGrid produces roughly same count as one
TEST(CloudMerger, TwoIdenticalCloudsMergedVoxelDedup) {
  auto cloud = makeTestCloud(1000, 0.05f);

  // Concatenate two identical clouds
  auto merged = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
  *merged = *cloud + *cloud;

  EXPECT_EQ(merged->size(), 2000u);

  // VoxelGrid dedup
  auto deduped = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
  pcl::VoxelGrid<pcl::PointXYZ> voxel;
  voxel.setInputCloud(merged);
  voxel.setLeafSize(0.002f, 0.002f, 0.002f);
  voxel.filter(*deduped);

  // VoxelGrid on single cloud
  auto single_deduped = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
  voxel.setInputCloud(cloud);
  voxel.filter(*single_deduped);

  // Merged+deduped should be roughly the same size as single deduped
  // Allow some tolerance due to voxel boundary effects
  double ratio = static_cast<double>(deduped->size()) /
                 static_cast<double>(single_deduped->size());
  EXPECT_NEAR(ratio, 1.0, 0.15);
}
