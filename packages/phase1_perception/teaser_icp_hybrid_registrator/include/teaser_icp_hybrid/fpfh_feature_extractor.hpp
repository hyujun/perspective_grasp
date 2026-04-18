#pragma once

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/features/fpfh_omp.h>

#include "teaser_icp_hybrid/registration_result.hpp"

namespace perspective_grasp {

/// Extracts FPFH features from a point cloud for TEASER++ correspondence matching
class FpfhFeatureExtractor {
 public:
  explicit FpfhFeatureExtractor(double normal_radius, double feature_radius);

  /// Extract FPFH features from input cloud
  /// @param cloud Input point cloud (will be downsampled internally if needed)
  /// @return FPFH feature descriptors, one per point
  pcl::PointCloud<pcl::FPFHSignature33>::Ptr extract(
      const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud) const;

 private:
  double normal_radius_;
  double feature_radius_;
};

}  // namespace perspective_grasp
