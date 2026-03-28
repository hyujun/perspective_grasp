#pragma once

#include <Eigen/Geometry>

namespace perspective_grasp {

/// Result of a TEASER++ + ICP hybrid registration
struct RegistrationResult {
  Eigen::Isometry3d transform{Eigen::Isometry3d::Identity()};
  double teaser_fitness{0.0};
  double icp_fitness{0.0};
  bool success{false};
  double elapsed_ms{0.0};
};

/// Configuration for the hybrid registration pipeline
struct RegistrationConfig {
  // TEASER++ parameters
  double noise_bound{0.05};
  double cbar2{1.0};
  bool estimate_scaling{false};
  int rotation_max_iterations{100};
  double rotation_cost_threshold{1e-6};

  // FPFH feature extraction
  double fpfh_normal_radius{0.015};
  double fpfh_feature_radius{0.025};

  // Voxel downsampling
  double voxel_size{0.005};

  // ICP local refinement
  int icp_max_iterations{50};
  double icp_transformation_epsilon{1e-8};
  double icp_euclidean_fitness_epsilon{1e-6};
  double icp_max_correspondence_distance{0.02};
};

}  // namespace perspective_grasp
