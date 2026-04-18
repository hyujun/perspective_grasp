#pragma once

#include <random>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace perspective_grasp::test_helpers {

/// Dense cube surface cloud (6 faces) centered at origin.
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

}  // namespace perspective_grasp::test_helpers
