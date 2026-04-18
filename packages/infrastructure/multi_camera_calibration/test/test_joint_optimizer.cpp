// Copyright 2026 perspective_grasp
// SPDX-License-Identifier: Apache-2.0

#include "multi_camera_calibration/joint_optimizer.hpp"

#include <gtest/gtest.h>

#include <Eigen/Geometry>

#include <cmath>
#include <random>
#include <vector>

namespace {

using perspective_grasp::CalibrationSample;
using perspective_grasp::CameraIntrinsics;
using perspective_grasp::JointOptimizer;
using perspective_grasp::JointResult;

CameraIntrinsics makeIntrinsics() {
  CameraIntrinsics intr;
  intr.fx = 600.0;
  intr.fy = 600.0;
  intr.cx = 320.0;
  intr.cy = 240.0;
  return intr;
}

}  // namespace

// --- Always-on behaviour -----------------------------------------------------

TEST(JointOptimizerTest, IsAvailableMatchesBuildFlag) {
#ifdef HAS_CERES
  EXPECT_TRUE(JointOptimizer::isAvailable());
#else
  EXPECT_FALSE(JointOptimizer::isAvailable());
#endif
}

TEST(JointOptimizerTest, EmptySamplesReturnsFailure) {
  JointOptimizer optimizer;
  std::vector<CalibrationSample> samples;
  std::map<int, CameraIntrinsics> intrinsics{{0, makeIntrinsics()}};

  JointResult result = optimizer.optimize(samples, intrinsics, 1);
  EXPECT_FALSE(result.converged);
  EXPECT_FALSE(result.message.empty());
#ifndef HAS_CERES
  EXPECT_NE(result.message.find("Ceres"), std::string::npos);
#endif
}

#ifndef HAS_CERES
TEST(JointOptimizerTest, StubReturnsCeresUnavailableMessage) {
  JointOptimizer optimizer;
  CalibrationSample sample;
  sample.camera_id = 0;
  sample.points_3d.emplace_back(0.1, 0.0, 0.0);
  sample.points_2d.emplace_back(100.0, 100.0);
  std::vector<CalibrationSample> samples{sample};
  std::map<int, CameraIntrinsics> intrinsics{{0, makeIntrinsics()}};

  JointResult result = optimizer.optimize(samples, intrinsics, 1);
  EXPECT_FALSE(result.converged);
  EXPECT_NE(result.message.find("Ceres"), std::string::npos)
      << "Stub must advertise that Ceres is missing";
}
#endif  // !HAS_CERES

// --- Convergence on synthetic data (Ceres-gated) -----------------------------

#ifdef HAS_CERES

namespace {

Eigen::Isometry3d makeTransform(double rx, double ry, double rz, double tx,
                                double ty, double tz) {
  Eigen::Isometry3d T = Eigen::Isometry3d::Identity();
  T.linear() = (Eigen::AngleAxisd(rz, Eigen::Vector3d::UnitZ()) *
                Eigen::AngleAxisd(ry, Eigen::Vector3d::UnitY()) *
                Eigen::AngleAxisd(rx, Eigen::Vector3d::UnitX()))
                   .toRotationMatrix();
  T.translation() = Eigen::Vector3d(tx, ty, tz);
  return T;
}

/// Project a 3D point in camera frame through pinhole intrinsics.
Eigen::Vector2d project(const Eigen::Vector3d& p_cam,
                        const CameraIntrinsics& intr) {
  return Eigen::Vector2d(intr.fx * p_cam.x() / p_cam.z() + intr.cx,
                         intr.fy * p_cam.y() / p_cam.z() + intr.cy);
}

}  // namespace

TEST(JointOptimizerTest, ConvergesOnSyntheticSingleCamera) {
  // Ground truth
  Eigen::Isometry3d T_cam_base_gt = makeTransform(0.1, -0.05, 0.2, 0.2, 0.3, 0.8);
  Eigen::Isometry3d T_board_ee_gt = makeTransform(0.0, 0.0, 0.0, 0.0, 0.0, 0.05);

  CameraIntrinsics intr = makeIntrinsics();

  // Board corners in board frame (simple 3x3 grid on a 0.1m plane)
  std::vector<Eigen::Vector3d> board_points;
  for (int i = 0; i < 3; ++i) {
    for (int j = 0; j < 3; ++j) {
      board_points.emplace_back(0.05 * i, 0.05 * j, 0.0);
    }
  }

  std::mt19937 gen(7);
  std::uniform_real_distribution<double> ang(-M_PI / 6.0, M_PI / 6.0);
  std::uniform_real_distribution<double> trans(-0.1, 0.1);

  std::vector<CalibrationSample> samples;
  constexpr int kNumSamples = 25;
  for (int s = 0; s < kNumSamples; ++s) {
    CalibrationSample sample;
    sample.camera_id = 0;
    sample.T_base_gripper = makeTransform(
        ang(gen), ang(gen), ang(gen), 0.5 + trans(gen), trans(gen), 0.3 + trans(gen));
    sample.points_3d = board_points;

    for (const auto& p_board : board_points) {
      Eigen::Vector3d p_ee = T_board_ee_gt * p_board;
      Eigen::Vector3d p_base = sample.T_base_gripper * p_ee;
      Eigen::Vector3d p_cam = T_cam_base_gt * p_base;
      ASSERT_GT(p_cam.z(), 0.0)
          << "Synthetic setup must keep points in front of camera";
      sample.points_2d.push_back(project(p_cam, intr));
    }
    samples.push_back(std::move(sample));
  }

  JointOptimizer optimizer;
  JointResult result =
      optimizer.optimize(samples, {{0, intr}}, /*num_cameras=*/1);

  EXPECT_TRUE(result.converged)
      << "Noise-free synthetic data should converge. Report: " << result.message;

  ASSERT_EQ(result.T_cam_base.count(0), 1U);
  const Eigen::Isometry3d& T_cam_base_est = result.T_cam_base.at(0);

  // Translation recovered within a few mm, rotation within a few millirad.
  EXPECT_LT((T_cam_base_est.translation() - T_cam_base_gt.translation()).norm(),
            0.01);

  Eigen::Matrix3d delta_R =
      T_cam_base_gt.rotation().transpose() * T_cam_base_est.rotation();
  Eigen::AngleAxisd aa(delta_R);
  EXPECT_LT(std::abs(aa.angle()), 0.02);

  // Final cost should be ~0 for noise-free data.
  EXPECT_LT(result.final_error, 1e-4);
}

TEST(JointOptimizerTest, HandlesMissingIntrinsicsByWarning) {
  // Sample references camera_id=5 but intrinsics only provided for 0.
  // The optimizer should skip that sample and still run (just with no data).
  CalibrationSample sample;
  sample.camera_id = 5;
  sample.points_3d.emplace_back(0.0, 0.0, 0.0);
  sample.points_2d.emplace_back(100.0, 100.0);

  JointOptimizer optimizer;
  std::map<int, CameraIntrinsics> intrinsics{{0, makeIntrinsics()}};
  JointResult result =
      optimizer.optimize({sample}, intrinsics, /*num_cameras=*/1);
  // No crash, result populated. With no residuals added, Ceres reports
  // "convergence" trivially — just verify no exception and T_cam_base[0]
  // entry exists.
  EXPECT_EQ(result.T_cam_base.count(0), 1U);
}

#endif  // HAS_CERES
