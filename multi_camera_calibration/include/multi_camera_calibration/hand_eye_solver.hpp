// Copyright 2026 perspective_grasp
// SPDX-License-Identifier: Apache-2.0

#pragma once

#include <Eigen/Geometry>

#include <vector>

namespace perspective_grasp {

/// Result of a hand-eye calibration solve.
struct CalibrationResult {
  Eigen::Isometry3d T_cam_base = Eigen::Isometry3d::Identity();
  double reprojection_error = 0.0;
  bool success = false;
};

/// Wraps OpenCV calibrateHandEye for eye-in-hand and eye-to-hand setups.
class HandEyeSolver {
 public:
  HandEyeSolver() = default;

  /// Solve eye-in-hand calibration (camera mounted on end-effector).
  /// @param T_base_gripper Transforms from base to gripper for each sample.
  /// @param T_target_cam Transforms from calibration target to camera for each
  /// sample.
  /// @return Calibration result with T_cam_base.
  [[nodiscard]] CalibrationResult solveEyeInHand(
      const std::vector<Eigen::Isometry3d>& T_base_gripper,
      const std::vector<Eigen::Isometry3d>& T_target_cam) const;

  /// Solve eye-to-hand calibration (camera fixed in world).
  /// @param T_gripper_base Transforms from gripper to base for each sample
  ///   (inverse of T_base_gripper).
  /// @param T_target_cam Transforms from calibration target to camera for each
  /// sample.
  /// @return Calibration result with T_cam_base.
  [[nodiscard]] CalibrationResult solveEyeToHand(
      const std::vector<Eigen::Isometry3d>& T_gripper_base,
      const std::vector<Eigen::Isometry3d>& T_target_cam) const;

};

}  // namespace perspective_grasp
