// Copyright 2026 perspective_grasp
// SPDX-License-Identifier: Apache-2.0

#pragma once

#include <Eigen/Geometry>

namespace perspective_grasp::detail {

/// 6-DoF pose packed as [angle_axis(3), translation(3)] — the parameter
/// layout expected by Ceres' AngleAxisRotatePoint. Lives in a detail header
/// so it can be unit-tested independently of the Ceres build.
struct Pose6D {
  double data[6] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

  static Pose6D fromIsometry(const Eigen::Isometry3d& T) {
    Pose6D pose;
    Eigen::AngleAxisd aa(T.rotation());
    Eigen::Vector3d axis_angle = aa.angle() * aa.axis();
    pose.data[0] = axis_angle.x();
    pose.data[1] = axis_angle.y();
    pose.data[2] = axis_angle.z();
    pose.data[3] = T.translation().x();
    pose.data[4] = T.translation().y();
    pose.data[5] = T.translation().z();
    return pose;
  }

  [[nodiscard]] Eigen::Isometry3d toIsometry() const {
    Eigen::Vector3d axis_angle(data[0], data[1], data[2]);
    double angle = axis_angle.norm();
    Eigen::Isometry3d T = Eigen::Isometry3d::Identity();
    if (angle > 1e-10) {
      T.linear() =
          Eigen::AngleAxisd(angle, axis_angle / angle).toRotationMatrix();
    }
    T.translation() = Eigen::Vector3d(data[3], data[4], data[5]);
    return T;
  }
};

}  // namespace perspective_grasp::detail
