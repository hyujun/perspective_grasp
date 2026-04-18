#include <gtest/gtest.h>

#include <pcl/filters/voxel_grid.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <memory>
#include <random>

#include "pcl_merge_node/cloud_preprocessor.hpp"

using perspective_grasp::CloudPreprocessor;
using perspective_grasp::PreprocessorParams;

namespace {

pcl::PointCloud<pcl::PointXYZ>::Ptr makeTestCloud(std::size_t n, float z_base) {
  auto cloud = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
  cloud->reserve(n);
  for (std::size_t i = 0; i < n; ++i) {
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

pcl::PointCloud<pcl::PointXYZ>::Ptr makeDenseInRangeCloud(std::size_t n) {
  // All points within [-0.05, 0.30] Z range for default params.
  auto cloud = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
  cloud->reserve(n);
  for (std::size_t i = 0; i < n; ++i) {
    float x = static_cast<float>(i % 50) * 0.002f;
    float y = static_cast<float>(i / 50) * 0.002f;
    cloud->push_back(pcl::PointXYZ(x, y, 0.10f));
  }
  cloud->width = static_cast<uint32_t>(cloud->size());
  cloud->height = 1;
  cloud->is_dense = true;
  return cloud;
}

}  // namespace

TEST(CloudPreprocessor, EmptyInputReturnsEmptyCloud) {
  PreprocessorParams params;
  CloudPreprocessor pp(params);

  auto cloud = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
  auto result = pp.process(cloud);
  ASSERT_NE(result, nullptr);
  EXPECT_TRUE(result->empty());
}

TEST(CloudPreprocessor, NullInputReturnsEmptyCloud) {
  PreprocessorParams params;
  CloudPreprocessor pp(params);

  pcl::PointCloud<pcl::PointXYZ>::Ptr null_cloud;
  auto result = pp.process(null_cloud);
  ASSERT_NE(result, nullptr);
  EXPECT_TRUE(result->empty());
}

TEST(CloudPreprocessor, AllPointsOutsideZRangeYieldEmptyResult) {
  PreprocessorParams params;
  params.table_height = 0.0;
  params.max_object_height = 0.30;
  CloudPreprocessor pp(params);

  auto cloud = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
  for (int i = 0; i < 200; ++i) {
    cloud->push_back(pcl::PointXYZ(0.0f, 0.0f, 2.0f));  // way above
  }
  cloud->width = static_cast<uint32_t>(cloud->size());
  cloud->height = 1;
  cloud->is_dense = true;

  auto result = pp.process(cloud);
  EXPECT_TRUE(result->empty());
}

TEST(CloudPreprocessor, MixedRangeFiltersOutOfRangePoints) {
  PreprocessorParams params;
  params.table_height = 0.0;
  params.max_object_height = 0.30;
  params.outlier_mean_k = 10;
  params.outlier_stddev = 2.0;
  CloudPreprocessor pp(params);

  auto cloud = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
  // 500 points in range, 200 out-of-range
  for (int i = 0; i < 500; ++i) {
    float x = static_cast<float>(i % 50) * 0.002f;
    float y = static_cast<float>(i / 50) * 0.002f;
    cloud->push_back(pcl::PointXYZ(x, y, 0.10f));
  }
  for (int i = 0; i < 100; ++i) {
    cloud->push_back(pcl::PointXYZ(0.0f, 0.0f, 1.0f));
  }
  for (int i = 0; i < 100; ++i) {
    cloud->push_back(pcl::PointXYZ(0.0f, 0.0f, -0.5f));
  }
  cloud->width = static_cast<uint32_t>(cloud->size());
  cloud->height = 1;
  cloud->is_dense = true;

  auto result = pp.process(cloud);
  EXPECT_LT(result->size(), cloud->size());
  EXPECT_LE(result->size(), 500u);
  EXPECT_GT(result->size(), 0u);
}

TEST(CloudPreprocessor, StatisticalOutlierRemovalDropsNoisePoints) {
  // Dense cluster + sparse outliers far from it.
  PreprocessorParams params;
  params.table_height = 0.0;
  params.max_object_height = 0.30;
  params.outlier_mean_k = 20;
  params.outlier_stddev = 1.0;  // aggressive
  CloudPreprocessor pp(params);

  auto cloud = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
  std::mt19937 rng(7);
  std::normal_distribution<float> noise(0.0f, 0.0005f);
  // 1000 tight points around (0,0,0.1)
  for (int i = 0; i < 1000; ++i) {
    cloud->push_back(pcl::PointXYZ(noise(rng), noise(rng), 0.10f + noise(rng)));
  }
  // 10 wild outliers within Z range but far in XY
  for (int i = 0; i < 10; ++i) {
    cloud->push_back(pcl::PointXYZ(0.15f + 0.01f * static_cast<float>(i),
                                    0.15f, 0.10f));
  }
  cloud->width = static_cast<uint32_t>(cloud->size());
  cloud->height = 1;
  cloud->is_dense = true;

  auto result = pp.process(cloud);
  // SOR should drop at least some of the outliers.
  EXPECT_LT(result->size(), cloud->size());
  // But not empty.
  EXPECT_GT(result->size(), 500u);
}

TEST(CloudPreprocessor, SetParamsReflectsInNextProcess) {
  // Start with narrow range, then widen via setParams.
  PreprocessorParams tight;
  tight.table_height = 0.0;
  tight.max_object_height = 0.05;  // only 5cm of height
  tight.outlier_mean_k = 10;
  tight.outlier_stddev = 3.0;
  CloudPreprocessor pp(tight);

  auto cloud = makeDenseInRangeCloud(500);  // all at z=0.10, outside tight
  auto tight_result = pp.process(cloud);

  PreprocessorParams wide;
  wide.table_height = 0.0;
  wide.max_object_height = 0.30;
  wide.outlier_mean_k = 10;
  wide.outlier_stddev = 3.0;
  pp.setParams(wide);
  auto wide_result = pp.process(cloud);

  EXPECT_LT(tight_result->size(), wide_result->size());
  EXPECT_GT(wide_result->size(), 0u);
}

TEST(CloudPreprocessor, TableHeightShiftsZWindow) {
  // Raise table_height: cloud at old Z should now fall out of range.
  PreprocessorParams params;
  params.table_height = 0.0;
  params.max_object_height = 0.30;
  params.outlier_mean_k = 10;
  params.outlier_stddev = 3.0;
  CloudPreprocessor pp(params);

  auto cloud = makeDenseInRangeCloud(400);  // z=0.10
  auto at_zero = pp.process(cloud);
  EXPECT_GT(at_zero->size(), 0u);

  params.table_height = 1.0;  // window becomes [0.95, 1.30]
  pp.setParams(params);
  auto at_one = pp.process(cloud);
  EXPECT_TRUE(at_one->empty());
}

TEST(CloudMerger, TwoIdenticalCloudsMergedVoxelDedup) {
  // Integration: merging two identical clouds and VoxelGrid-dedupe yields
  // roughly the same count as a single deduped cloud.
  auto cloud = makeTestCloud(1000, 0.05f);

  auto merged = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
  *merged = *cloud + *cloud;
  EXPECT_EQ(merged->size(), 2000u);

  auto deduped = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
  pcl::VoxelGrid<pcl::PointXYZ> voxel;
  voxel.setInputCloud(merged);
  voxel.setLeafSize(0.002f, 0.002f, 0.002f);
  voxel.filter(*deduped);

  auto single_deduped = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
  voxel.setInputCloud(cloud);
  voxel.filter(*single_deduped);

  double ratio = static_cast<double>(deduped->size()) /
                 static_cast<double>(single_deduped->size());
  EXPECT_NEAR(ratio, 1.0, 0.15);
}
