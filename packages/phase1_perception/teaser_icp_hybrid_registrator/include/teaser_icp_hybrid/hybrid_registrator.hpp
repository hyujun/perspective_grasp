#pragma once

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include "teaser_icp_hybrid/fpfh_feature_extractor.hpp"
#include "teaser_icp_hybrid/registration_result.hpp"

namespace perspective_grasp {

/// Hybrid point cloud registration combining TEASER++ global registration
/// with PCL ICP local refinement.
///
/// Usage:
///   HybridRegistrator reg(config);
///   // Full pipeline (no initial guess) - for first detection or re-init:
///   auto result = reg.align(cad_cloud, scene_cloud);
///   // ICP-only (with initial guess) - for frame-to-frame tracking:
///   auto result = reg.refineIcp(cad_cloud, scene_cloud, prev_pose);
class HybridRegistrator {
 public:
  explicit HybridRegistrator(const RegistrationConfig& config = {});

  /// Full pipeline: FPFH extraction -> TEASER++ global -> ICP local refinement
  /// @param source CAD model point cloud
  /// @param target Scene (sensor) point cloud
  /// @return Registration result with combined transform and fitness scores
  RegistrationResult align(
      const pcl::PointCloud<pcl::PointXYZ>::Ptr& source,
      const pcl::PointCloud<pcl::PointXYZ>::Ptr& target) const;

  /// ICP-only refinement with a known initial guess
  /// @param source CAD model point cloud
  /// @param target Scene (sensor) point cloud
  /// @param initial_guess Previous frame pose or TEASER++ result
  /// @return Registration result with refined transform and ICP fitness
  RegistrationResult refineIcp(
      const pcl::PointCloud<pcl::PointXYZ>::Ptr& source,
      const pcl::PointCloud<pcl::PointXYZ>::Ptr& target,
      const Eigen::Isometry3d& initial_guess) const;

  void updateConfig(const RegistrationConfig& config);

 private:
  RegistrationConfig config_;
  std::unique_ptr<FpfhFeatureExtractor> feature_extractor_;

  /// Downsample a point cloud using voxel grid filter
  pcl::PointCloud<pcl::PointXYZ>::Ptr downsample(
      const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud) const;
};

}  // namespace perspective_grasp
