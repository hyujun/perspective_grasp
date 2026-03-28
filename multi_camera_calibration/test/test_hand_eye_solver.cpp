// Copyright 2026 perspective_grasp
// SPDX-License-Identifier: Apache-2.0

#include "multi_camera_calibration/hand_eye_solver.hpp"

#include <gtest/gtest.h>

#include <Eigen/Geometry>

#include <cmath>
#include <random>
#include <vector>

namespace {

/// Generate a random rotation (small perturbation).
Eigen::Matrix3d randomRotation(std::mt19937& gen, double max_angle_rad) {
  std::uniform_real_distribution<double> dist(-1.0, 1.0);
  Eigen::Vector3d axis(dist(gen), dist(gen), dist(gen));
  axis.normalize();
  std::uniform_real_distribution<double> angle_dist(0.0, max_angle_rad);
  return Eigen::AngleAxisd(angle_dist(gen), axis).toRotationMatrix();
}

/// Generate a random translation.
Eigen::Vector3d randomTranslation(std::mt19937& gen, double max_t) {
  std::uniform_real_distribution<double> dist(-max_t, max_t);
  return Eigen::Vector3d(dist(gen), dist(gen), dist(gen));
}

/// Generate synthetic hand-eye calibration data.
/// Given a known T_cam_gripper, generates N robot poses and corresponding
/// board observations.
void generateSyntheticData(
    const Eigen::Isometry3d& T_cam_gripper,
    const Eigen::Isometry3d& T_board_base, int num_samples,
    std::vector<Eigen::Isometry3d>& T_base_gripper_out,
    std::vector<Eigen::Isometry3d>& T_target_cam_out) {
  std::mt19937 gen(42);  // Fixed seed for reproducibility

  T_base_gripper_out.clear();
  T_target_cam_out.clear();
  T_base_gripper_out.reserve(static_cast<size_t>(num_samples));
  T_target_cam_out.reserve(static_cast<size_t>(num_samples));

  for (int i = 0; i < num_samples; ++i) {
    // Random gripper pose in base frame
    Eigen::Isometry3d T_base_gripper = Eigen::Isometry3d::Identity();
    T_base_gripper.linear() = randomRotation(gen, M_PI / 4.0);
    T_base_gripper.translation() = randomTranslation(gen, 0.3);
    // Shift to a plausible workspace position
    T_base_gripper.translation() += Eigen::Vector3d(0.5, 0.0, 0.5);

    // Compute what the camera sees:
    // T_target_cam = T_cam_gripper^{-1} * T_base_gripper^{-1} * T_board_base
    // But actually for eye-in-hand:
    //   camera is on gripper -> T_cam_base = T_cam_gripper * T_gripper_base
    //   T_target_cam = T_cam_base * T_base_board^{-1}... let's be precise:
    //
    // The board is at T_board_base (board -> base transform).
    // The camera is at T_cam_gripper relative to the gripper.
    // The gripper is at T_base_gripper relative to base.
    //
    // T_cam_world = T_cam_gripper * T_gripper_base
    //             = T_cam_gripper * T_base_gripper.inverse()
    //
    // T_target_cam = T_cam_world * T_world_board
    //              = T_cam_gripper * T_base_gripper.inverse() * T_board_base.inverse()
    //
    // Wait, T_board_base maps board->base, so base->board = T_board_base.inverse()
    // Actually let's define clearly:
    //   T_board_base = transform that takes a point in board frame to base frame
    //   T_target_cam = transform that takes a point in target/board frame to cam frame
    //
    // p_cam = T_cam_gripper * T_gripper_base * p_base
    //       = T_cam_gripper * inv(T_base_gripper) * p_base
    //
    // p_base = T_board_base * p_board
    //
    // So: p_cam = T_cam_gripper * inv(T_base_gripper) * T_board_base * p_board
    // Therefore: T_target_cam = T_cam_gripper * inv(T_base_gripper) * T_board_base

    Eigen::Isometry3d T_target_cam =
        T_cam_gripper * T_base_gripper.inverse() * T_board_base;

    T_base_gripper_out.push_back(T_base_gripper);
    T_target_cam_out.push_back(T_target_cam);
  }
}

}  // namespace

TEST(HandEyeSolverTest, EyeInHandSyntheticData) {
  // Known ground truth: camera mounted on gripper
  Eigen::Isometry3d T_cam_gripper_gt = Eigen::Isometry3d::Identity();
  T_cam_gripper_gt.linear() =
      Eigen::AngleAxisd(0.1, Eigen::Vector3d::UnitX()).toRotationMatrix() *
      Eigen::AngleAxisd(-0.05, Eigen::Vector3d::UnitY()).toRotationMatrix();
  T_cam_gripper_gt.translation() = Eigen::Vector3d(0.05, -0.02, 0.08);

  // Board fixed in base frame
  Eigen::Isometry3d T_board_base = Eigen::Isometry3d::Identity();
  T_board_base.linear() =
      Eigen::AngleAxisd(0.3, Eigen::Vector3d::UnitZ()).toRotationMatrix();
  T_board_base.translation() = Eigen::Vector3d(0.6, 0.1, 0.0);

  // Generate 30 synthetic samples
  constexpr int kNumSamples = 30;
  std::vector<Eigen::Isometry3d> T_base_gripper, T_target_cam;
  generateSyntheticData(T_cam_gripper_gt, T_board_base, kNumSamples,
                        T_base_gripper, T_target_cam);

  ASSERT_EQ(T_base_gripper.size(), static_cast<size_t>(kNumSamples));

  // Solve
  perspective_grasp::HandEyeSolver solver;
  auto result = solver.solveEyeInHand(T_base_gripper, T_target_cam);

  EXPECT_TRUE(result.success)
      << "Hand-eye solver should succeed with synthetic data";

  // Verify the result is a valid SE(3) transform (not NaN/Inf)
  if (result.success) {
    EXPECT_TRUE(result.T_cam_base.translation().allFinite());
    EXPECT_GT(result.T_cam_base.rotation().determinant(), 0.9);
  }
}

TEST(HandEyeSolverTest, EyeToHandSyntheticData) {
  // Known ground truth: camera fixed in world, board on gripper
  Eigen::Isometry3d T_cam_base_gt = Eigen::Isometry3d::Identity();
  T_cam_base_gt.linear() =
      Eigen::AngleAxisd(-0.2, Eigen::Vector3d::UnitY()).toRotationMatrix() *
      Eigen::AngleAxisd(0.15, Eigen::Vector3d::UnitX()).toRotationMatrix();
  T_cam_base_gt.translation() = Eigen::Vector3d(0.0, 0.5, 1.0);

  // Board attached to gripper (T_board_gripper)
  Eigen::Isometry3d T_board_gripper = Eigen::Isometry3d::Identity();
  T_board_gripper.translation() = Eigen::Vector3d(0.0, 0.0, 0.05);

  constexpr int kNumSamples = 30;
  std::mt19937 gen(123);

  std::vector<Eigen::Isometry3d> T_gripper_base_vec, T_target_cam_vec;

  for (int i = 0; i < kNumSamples; ++i) {
    Eigen::Isometry3d T_base_gripper = Eigen::Isometry3d::Identity();
    T_base_gripper.linear() = randomRotation(gen, M_PI / 4.0);
    T_base_gripper.translation() = randomTranslation(gen, 0.3);
    T_base_gripper.translation() += Eigen::Vector3d(0.5, 0.0, 0.3);

    // T_target_cam = T_cam_base * T_base_gripper * T_board_gripper
    // (board is on gripper, camera sees board from fixed position)
    Eigen::Isometry3d T_target_cam =
        T_cam_base_gt * T_base_gripper * T_board_gripper;

    // For eye-to-hand, we pass T_gripper_base = inv(T_base_gripper)
    T_gripper_base_vec.push_back(T_base_gripper.inverse());
    T_target_cam_vec.push_back(T_target_cam);
  }

  perspective_grasp::HandEyeSolver solver;
  auto result = solver.solveEyeToHand(T_gripper_base_vec, T_target_cam_vec);

  EXPECT_TRUE(result.success);

  // Verify valid SE(3) output
  if (result.success) {
    EXPECT_TRUE(result.T_cam_base.translation().allFinite());
    EXPECT_GT(result.T_cam_base.rotation().determinant(), 0.9);
  }
}

TEST(HandEyeSolverTest, RejectsTooFewSamples) {
  perspective_grasp::HandEyeSolver solver;

  std::vector<Eigen::Isometry3d> T_base_gripper(2,
                                                  Eigen::Isometry3d::Identity());
  std::vector<Eigen::Isometry3d> T_target_cam(2,
                                               Eigen::Isometry3d::Identity());

  auto result = solver.solveEyeInHand(T_base_gripper, T_target_cam);
  EXPECT_FALSE(result.success) << "Should fail with only 2 samples";
}

TEST(HandEyeSolverTest, RejectsMismatchedSizes) {
  perspective_grasp::HandEyeSolver solver;

  std::vector<Eigen::Isometry3d> T_base_gripper(5,
                                                  Eigen::Isometry3d::Identity());
  std::vector<Eigen::Isometry3d> T_target_cam(3,
                                               Eigen::Isometry3d::Identity());

  auto result = solver.solveEyeInHand(T_base_gripper, T_target_cam);
  EXPECT_FALSE(result.success) << "Should fail with mismatched sizes";
}
