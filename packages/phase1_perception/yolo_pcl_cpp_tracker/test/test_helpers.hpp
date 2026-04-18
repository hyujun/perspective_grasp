#pragma once

#include <cmath>
#include <random>
#include <vector>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace perspective_grasp::test_helpers {

/// Dense cube surface cloud centered at origin (6 faces).
inline pcl::PointCloud<pcl::PointXYZ>::Ptr makeCubeCloud(
    double side_length, int points_per_face) {
  auto cloud = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
  const double half = side_length / 2.0;
  const double step =
      side_length / static_cast<double>(std::max(1, points_per_face - 1));

  for (int i = 0; i < points_per_face; ++i) {
    for (int j = 0; j < points_per_face; ++j) {
      const double u = -half + step * static_cast<double>(i);
      const double v = -half + step * static_cast<double>(j);
      cloud->emplace_back(static_cast<float>(u), static_cast<float>(v),
                          static_cast<float>(half));
      cloud->emplace_back(static_cast<float>(u), static_cast<float>(v),
                          static_cast<float>(-half));
      cloud->emplace_back(static_cast<float>(u), static_cast<float>(half),
                          static_cast<float>(v));
      cloud->emplace_back(static_cast<float>(u), static_cast<float>(-half),
                          static_cast<float>(v));
      cloud->emplace_back(static_cast<float>(half), static_cast<float>(u),
                          static_cast<float>(v));
      cloud->emplace_back(static_cast<float>(-half), static_cast<float>(u),
                          static_cast<float>(v));
    }
  }
  cloud->width = static_cast<uint32_t>(cloud->size());
  cloud->height = 1;
  cloud->is_dense = true;
  return cloud;
}

/// Add iid Gaussian noise to every point in place.
inline void addGaussianNoise(pcl::PointCloud<pcl::PointXYZ>& cloud,
                             double stddev, uint32_t seed = 42) {
  std::mt19937 rng(seed);
  std::normal_distribution<float> dist(0.0f, static_cast<float>(stddev));
  for (auto& pt : cloud) {
    pt.x += dist(rng);
    pt.y += dist(rng);
    pt.z += dist(rng);
  }
}

/// Build an organized (height>1) cloud where each (x,y) index maps to a 3D
/// point with depth z_center + (x+y)*z_slope — useful for ROI crop tests.
inline pcl::PointCloud<pcl::PointXYZ>::Ptr makeOrganizedCloud(
    uint32_t width, uint32_t height, float z_center = 1.0f,
    float z_slope = 0.001f) {
  auto cloud = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
  cloud->width = width;
  cloud->height = height;
  cloud->is_dense = false;
  cloud->points.resize(static_cast<size_t>(width) * height);
  for (uint32_t y = 0; y < height; ++y) {
    for (uint32_t x = 0; x < width; ++x) {
      auto& pt = cloud->at(x, y);
      pt.x = static_cast<float>(x) * 0.001f;
      pt.y = static_cast<float>(y) * 0.001f;
      pt.z = z_center + z_slope * static_cast<float>(x + y);
    }
  }
  return cloud;
}

}  // namespace perspective_grasp::test_helpers
