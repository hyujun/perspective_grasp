#include <gtest/gtest.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <memory>

#include "pcl_merge_node/overlap_filter.hpp"

using perspective_grasp::OverlapFilter;

namespace {

pcl::PointCloud<pcl::PointXYZ>::Ptr makeGrid(float x_offset, float y_offset,
                                              int nx, int ny, float spacing) {
  auto cloud = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
  for (int ix = 0; ix < nx; ++ix) {
    for (int iy = 0; iy < ny; ++iy) {
      float x = x_offset + static_cast<float>(ix) * spacing;
      float y = y_offset + static_cast<float>(iy) * spacing;
      cloud->push_back(pcl::PointXYZ(x, y, 0.0f));
    }
  }
  cloud->width = static_cast<uint32_t>(cloud->size());
  cloud->height = 1;
  cloud->is_dense = true;
  return cloud;
}

}  // namespace

TEST(OverlapFilter, NonOverlappingCloudsAllCountOne) {
  auto cloud_a = makeGrid(0.0f, 0.0f, 10, 10, 0.01f);
  auto cloud_b = makeGrid(1.0f, 0.0f, 10, 10, 0.01f);

  auto merged = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
  *merged = *cloud_a + *cloud_b;

  OverlapFilter filter(0.005);
  filter.process(merged, cloud_a, cloud_b);

  const auto& counts = filter.getOverlapCounts();
  ASSERT_EQ(counts.size(), merged->size());
  for (std::size_t i = 0; i < counts.size(); ++i) {
    EXPECT_EQ(counts[i], 1) << "Point " << i;
  }
}

TEST(OverlapFilter, IdenticalCloudsAllCountTwo) {
  auto cloud = makeGrid(0.0f, 0.0f, 10, 10, 0.01f);
  auto cloud_a = cloud;
  auto cloud_b = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>(*cloud);
  auto merged = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>(*cloud);

  OverlapFilter filter(0.005);
  filter.process(merged, cloud_a, cloud_b);

  const auto& counts = filter.getOverlapCounts();
  ASSERT_EQ(counts.size(), merged->size());
  for (std::size_t i = 0; i < counts.size(); ++i) {
    EXPECT_EQ(counts[i], 2);
  }
}

TEST(OverlapFilter, PartialOverlapMixedCounts) {
  // cloud_a covers [0, 0.1], cloud_b covers [0.05, 0.15].
  // Overlap region is [0.05, 0.10].
  auto cloud_a = makeGrid(0.00f, 0.00f, 11, 5, 0.01f);  // x in [0.00..0.10]
  auto cloud_b = makeGrid(0.05f, 0.00f, 11, 5, 0.01f);  // x in [0.05..0.15]

  auto merged = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
  *merged = *cloud_a + *cloud_b;

  OverlapFilter filter(0.001);  // tight radius — only exact matches count
  filter.process(merged, cloud_a, cloud_b);

  const auto& counts = filter.getOverlapCounts();
  int ones = 0, twos = 0;
  for (int c : counts) {
    if (c == 1) ++ones;
    else if (c == 2) ++twos;
  }
  // We expect *some* exclusive points (seen by one cloud only) and *some*
  // shared points (in the overlap region present in both source clouds).
  EXPECT_GT(ones, 0);
  EXPECT_GT(twos, 0);
  EXPECT_EQ(ones + twos, static_cast<int>(counts.size()));
}

TEST(OverlapFilter, EmptyMergedYieldsEmptyCounts) {
  auto cloud_a = makeGrid(0.0f, 0.0f, 5, 5, 0.01f);
  auto cloud_b = makeGrid(1.0f, 0.0f, 5, 5, 0.01f);

  auto merged = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();

  OverlapFilter filter(0.005);
  filter.process(merged, cloud_a, cloud_b);
  EXPECT_TRUE(filter.getOverlapCounts().empty());
}

TEST(OverlapFilter, NullMergedYieldsEmptyCounts) {
  auto cloud_a = makeGrid(0.0f, 0.0f, 5, 5, 0.01f);
  auto cloud_b = makeGrid(1.0f, 0.0f, 5, 5, 0.01f);

  pcl::PointCloud<pcl::PointXYZ>::Ptr merged;
  OverlapFilter filter(0.005);
  filter.process(merged, cloud_a, cloud_b);
  EXPECT_TRUE(filter.getOverlapCounts().empty());
}

TEST(OverlapFilter, OneSourceNullOnlyCountsValid) {
  // If cloud_b is null, every merged point should have count 1 (from cloud_a).
  auto cloud_a = makeGrid(0.0f, 0.0f, 5, 5, 0.01f);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_b;  // null
  auto merged = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>(*cloud_a);

  OverlapFilter filter(0.005);
  filter.process(merged, cloud_a, cloud_b);

  const auto& counts = filter.getOverlapCounts();
  ASSERT_EQ(counts.size(), merged->size());
  for (int c : counts) {
    EXPECT_EQ(c, 1);
  }
}

TEST(OverlapFilter, ShrinkingRadiusReducesOverlapCount) {
  // Two clouds offset by 2mm. With 5mm radius, all points overlap (count=2).
  // With 1mm radius, none do (count=1).
  auto cloud_a = makeGrid(0.0f, 0.0f, 10, 10, 0.01f);
  auto cloud_b = makeGrid(0.002f, 0.0f, 10, 10, 0.01f);
  auto merged = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
  *merged = *cloud_a + *cloud_b;

  OverlapFilter filter(0.005);
  filter.process(merged, cloud_a, cloud_b);
  int twos_loose = 0;
  for (int c : filter.getOverlapCounts()) if (c == 2) ++twos_loose;

  filter.setOverlapRadius(0.001);
  filter.process(merged, cloud_a, cloud_b);
  int twos_tight = 0;
  for (int c : filter.getOverlapCounts()) if (c == 2) ++twos_tight;

  EXPECT_GT(twos_loose, twos_tight);
}

TEST(OverlapFilter, GetOverlapCountIndexGuards) {
  auto cloud = makeGrid(0.0f, 0.0f, 3, 3, 0.01f);
  auto merged = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>(*cloud);

  OverlapFilter filter(0.005);
  filter.process(merged, cloud, cloud);

  // Valid index returns 1 or 2.
  int in_bounds = filter.getOverlapCount(0);
  EXPECT_GE(in_bounds, 1);
  // Out-of-range returns 0 per API contract.
  EXPECT_EQ(filter.getOverlapCount(9999), 0);
}
