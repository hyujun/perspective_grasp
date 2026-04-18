// Copyright 2026 perspective_grasp
// SPDX-License-Identifier: Apache-2.0

#pragma once

#include <Eigen/Geometry>

#include <map>
#include <string>
#include <vector>

namespace perspective_grasp {

/// Camera intrinsic parameters.
struct CameraIntrinsics {
  double fx = 0.0;
  double fy = 0.0;
  double cx = 0.0;
  double cy = 0.0;
  double dist_coeffs[5] = {0.0, 0.0, 0.0, 0.0, 0.0};
};

/// A single calibration data sample.
struct CalibrationSample {
  int camera_id = 0;
  Eigen::Isometry3d T_base_gripper = Eigen::Isometry3d::Identity();
  Eigen::Isometry3d T_target_cam = Eigen::Isometry3d::Identity();
  std::vector<Eigen::Vector3d> points_3d;
  std::vector<Eigen::Vector2d> points_2d;
};

/// Result of joint optimization across multiple cameras.
struct JointResult {
  std::map<int, Eigen::Isometry3d> T_cam_base;
  Eigen::Isometry3d T_board_ee = Eigen::Isometry3d::Identity();
  double final_error = 0.0;
  bool converged = false;
  std::string message;
};

/// Joint optimizer using Ceres Solver for multi-camera hand-eye calibration.
/// When Ceres is not available, all methods return a stub result indicating
/// that Ceres is not installed.
class JointOptimizer {
 public:
  JointOptimizer() = default;

  /// Run joint optimization over all calibration samples.
  /// @param samples Calibration data from all cameras.
  /// @param intrinsics Camera intrinsics indexed by camera id.
  /// @param num_cameras Number of cameras in the system.
  /// @return Optimization result.
  [[nodiscard]] JointResult optimize(
      const std::vector<CalibrationSample>& samples,
      const std::map<int, CameraIntrinsics>& intrinsics,
      int num_cameras) const;

  /// Check if Ceres optimization is available.
  [[nodiscard]] static bool isAvailable();
};

}  // namespace perspective_grasp
