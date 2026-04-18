// Copyright 2026 perspective_grasp
// SPDX-License-Identifier: Apache-2.0

#include "multi_camera_calibration/detail/pose6d.hpp"
#include "multi_camera_calibration/detail/pose_converters.hpp"

#include <gtest/gtest.h>

#include <Eigen/Geometry>

#include <cmath>
#include <random>

namespace {

using perspective_grasp::detail::Pose6D;
using perspective_grasp::detail::isometryToRvecTvec;
using perspective_grasp::detail::rvecTvecToIsometry;

constexpr double kTransformTol = 1e-9;

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

void expectIsometryNear(const Eigen::Isometry3d& a, const Eigen::Isometry3d& b,
                        double tol) {
  EXPECT_LT((a.translation() - b.translation()).norm(), tol);
  Eigen::Matrix3d delta = a.rotation().transpose() * b.rotation();
  Eigen::AngleAxisd aa(delta);
  EXPECT_LT(std::abs(aa.angle()), tol);
}

}  // namespace

// --- isometryToRvecTvec / rvecTvecToIsometry round-trip ---------------------

TEST(PoseConvertersTest, IdentityRoundTrip) {
  Eigen::Isometry3d T = Eigen::Isometry3d::Identity();
  cv::Mat rvec, tvec;
  isometryToRvecTvec(T, rvec, tvec);

  ASSERT_EQ(rvec.type(), CV_64F);
  ASSERT_EQ(tvec.type(), CV_64F);
  ASSERT_EQ(tvec.rows, 3);
  ASSERT_EQ(tvec.cols, 1);

  Eigen::Isometry3d T_round = rvecTvecToIsometry(rvec, tvec);
  expectIsometryNear(T, T_round, kTransformTol);
}

TEST(PoseConvertersTest, TranslationOnlyRoundTrip) {
  Eigen::Isometry3d T = Eigen::Isometry3d::Identity();
  T.translation() = Eigen::Vector3d(0.3, -1.2, 5.0);

  cv::Mat rvec, tvec;
  isometryToRvecTvec(T, rvec, tvec);

  EXPECT_NEAR(tvec.at<double>(0), 0.3, kTransformTol);
  EXPECT_NEAR(tvec.at<double>(1), -1.2, kTransformTol);
  EXPECT_NEAR(tvec.at<double>(2), 5.0, kTransformTol);

  Eigen::Isometry3d T_round = rvecTvecToIsometry(rvec, tvec);
  expectIsometryNear(T, T_round, kTransformTol);
}

TEST(PoseConvertersTest, RotationOnlyRoundTrip) {
  Eigen::Isometry3d T = makeTransform(0.3, -0.5, 1.2, 0.0, 0.0, 0.0);

  cv::Mat rvec, tvec;
  isometryToRvecTvec(T, rvec, tvec);

  Eigen::Isometry3d T_round = rvecTvecToIsometry(rvec, tvec);
  expectIsometryNear(T, T_round, kTransformTol);
}

TEST(PoseConvertersTest, RandomSamplesRoundTrip) {
  std::mt19937 gen(0xCAFE);
  std::uniform_real_distribution<double> angle(-M_PI, M_PI);
  std::uniform_real_distribution<double> trans(-5.0, 5.0);

  for (int i = 0; i < 50; ++i) {
    Eigen::Isometry3d T =
        makeTransform(angle(gen), angle(gen) * 0.5, angle(gen), trans(gen),
                      trans(gen), trans(gen));

    cv::Mat rvec, tvec;
    isometryToRvecTvec(T, rvec, tvec);
    Eigen::Isometry3d T_round = rvecTvecToIsometry(rvec, tvec);
    expectIsometryNear(T, T_round, 1e-9);
  }
}

TEST(PoseConvertersTest, ProducesValidRodriguesShape) {
  Eigen::Isometry3d T = makeTransform(0.1, 0.2, 0.3, 0.0, 0.0, 0.0);
  cv::Mat rvec, tvec;
  isometryToRvecTvec(T, rvec, tvec);

  EXPECT_EQ(rvec.total(), 3U);
  EXPECT_EQ(tvec.total(), 3U);
}

// --- Pose6D round-trip -------------------------------------------------------

TEST(Pose6DTest, DefaultIsIdentity) {
  Pose6D pose;
  Eigen::Isometry3d T = pose.toIsometry();
  expectIsometryNear(T, Eigen::Isometry3d::Identity(), kTransformTol);
}

TEST(Pose6DTest, IdentityRoundTrip) {
  Pose6D pose = Pose6D::fromIsometry(Eigen::Isometry3d::Identity());
  for (int i = 0; i < 6; ++i) {
    EXPECT_NEAR(pose.data[i], 0.0, kTransformTol)
        << "Identity should zero all 6 components, index " << i;
  }
}

TEST(Pose6DTest, TranslationStoredLast3) {
  Eigen::Isometry3d T = Eigen::Isometry3d::Identity();
  T.translation() = Eigen::Vector3d(1.0, 2.0, 3.0);

  Pose6D pose = Pose6D::fromIsometry(T);
  EXPECT_NEAR(pose.data[3], 1.0, kTransformTol);
  EXPECT_NEAR(pose.data[4], 2.0, kTransformTol);
  EXPECT_NEAR(pose.data[5], 3.0, kTransformTol);
  // Rotation block untouched
  EXPECT_NEAR(pose.data[0], 0.0, kTransformTol);
  EXPECT_NEAR(pose.data[1], 0.0, kTransformTol);
  EXPECT_NEAR(pose.data[2], 0.0, kTransformTol);
}

TEST(Pose6DTest, AxisAngleMagnitudeMatches) {
  Eigen::Isometry3d T = Eigen::Isometry3d::Identity();
  T.linear() =
      Eigen::AngleAxisd(0.7, Eigen::Vector3d::UnitZ()).toRotationMatrix();

  Pose6D pose = Pose6D::fromIsometry(T);
  Eigen::Vector3d aa(pose.data[0], pose.data[1], pose.data[2]);
  EXPECT_NEAR(aa.norm(), 0.7, kTransformTol);
  // Axis should be +Z
  EXPECT_NEAR(aa.normalized().z(), 1.0, kTransformTol);
}

TEST(Pose6DTest, RandomSamplesRoundTrip) {
  std::mt19937 gen(0xF00D);
  std::uniform_real_distribution<double> angle(-M_PI + 0.1, M_PI - 0.1);
  std::uniform_real_distribution<double> trans(-3.0, 3.0);

  for (int i = 0; i < 50; ++i) {
    Eigen::Isometry3d T =
        Eigen::Isometry3d::Identity();
    T.linear() = (Eigen::AngleAxisd(angle(gen), Eigen::Vector3d::UnitZ()) *
                  Eigen::AngleAxisd(angle(gen) * 0.5, Eigen::Vector3d::UnitY()))
                     .toRotationMatrix();
    T.translation() = Eigen::Vector3d(trans(gen), trans(gen), trans(gen));

    Pose6D pose = Pose6D::fromIsometry(T);
    Eigen::Isometry3d T_round = pose.toIsometry();
    expectIsometryNear(T, T_round, 1e-9);
  }
}

TEST(Pose6DTest, SmallAngleStaysIdentityRotation) {
  // Angles below 1e-10 branch should leave rotation at identity
  Pose6D pose;
  pose.data[0] = 1e-12;
  pose.data[1] = 1e-12;
  pose.data[2] = 1e-12;
  pose.data[3] = 0.5;

  Eigen::Isometry3d T = pose.toIsometry();
  EXPECT_TRUE(T.rotation().isApprox(Eigen::Matrix3d::Identity(), 1e-9));
  EXPECT_NEAR(T.translation().x(), 0.5, kTransformTol);
}
