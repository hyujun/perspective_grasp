#include <gtest/gtest.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

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

// Test 1: Two non-overlapping clouds - all overlap counts should be 1
TEST(OverlapFilter, NonOverlappingCloudsAllCountOne) {
  // cloud_a: grid at x=[0, 0.1], cloud_b: grid at x=[1.0, 1.1]
  // Well separated, no overlap
  auto cloud_a = makeGrid(0.0f, 0.0f, 10, 10, 0.01f);
  auto cloud_b = makeGrid(1.0f, 0.0f, 10, 10, 0.01f);

  // Merged = a + b
  auto merged = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
  *merged = *cloud_a + *cloud_b;

  OverlapFilter filter(0.005);  // 5mm radius
  filter.process(merged, cloud_a, cloud_b);

  const auto& counts = filter.getOverlapCounts();
  ASSERT_EQ(counts.size(), merged->size());

  for (size_t i = 0; i < counts.size(); ++i) {
    EXPECT_EQ(counts[i], 1) << "Point " << i << " has unexpected overlap count";
  }
}

// Test 2: Two identical clouds - all overlap counts should be 2
TEST(OverlapFilter, IdenticalCloudsAllCountTwo) {
  auto cloud = makeGrid(0.0f, 0.0f, 10, 10, 0.01f);

  // Both source clouds are identical
  auto cloud_a = cloud;
  auto cloud_b = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>(*cloud);

  // Merged is also the same set of points (simulating after VoxelGrid dedup)
  auto merged = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>(*cloud);

  OverlapFilter filter(0.005);
  filter.process(merged, cloud_a, cloud_b);

  const auto& counts = filter.getOverlapCounts();
  ASSERT_EQ(counts.size(), merged->size());

  for (size_t i = 0; i < counts.size(); ++i) {
    EXPECT_EQ(counts[i], 2) << "Point " << i << " should be seen by both cameras";
  }
}
