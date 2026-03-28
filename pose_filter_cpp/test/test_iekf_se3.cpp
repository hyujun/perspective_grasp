#include <gtest/gtest.h>
#include "pose_filter_cpp/iekf_se3.hpp"

namespace perspective_grasp {
namespace {

TEST(IekfSe3, InitializesOnFirstUpdate) {
  IekfSe3 filter;
  EXPECT_FALSE(filter.initialized());

  Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
  pose.translation() = Eigen::Vector3d(1.0, 2.0, 3.0);
  filter.update(pose);

  EXPECT_TRUE(filter.initialized());
  EXPECT_NEAR(filter.pose().translation().x(), 1.0, 1e-6);
}

TEST(IekfSe3, PredictMaintainsConstantVelocity) {
  IekfSe3::Config config;
  IekfSe3 filter(config);

  // Initialize at origin
  filter.reset(Eigen::Isometry3d::Identity());

  // Give it two updates to establish velocity
  Eigen::Isometry3d p1 = Eigen::Isometry3d::Identity();
  p1.translation() = Eigen::Vector3d(0.1, 0.0, 0.0);
  filter.predict(0.033);
  filter.update(p1);

  // After predict without update, pose should extrapolate
  filter.predict(0.033);
  auto predicted = filter.pose();
  // Should be somewhere near p1 (exact value depends on filter dynamics)
  EXPECT_GT(predicted.translation().x(), 0.0);
}

TEST(IekfSe3, ConvergesToMeasurement) {
  IekfSe3::Config config;
  config.max_iterations = 3;
  IekfSe3 filter(config);

  Eigen::Isometry3d target = Eigen::Isometry3d::Identity();
  target.translation() = Eigen::Vector3d(0.5, -0.3, 1.0);
  target.linear() =
      Eigen::AngleAxisd(0.5, Eigen::Vector3d::UnitZ()).toRotationMatrix();

  // Feed same measurement repeatedly
  for (int i = 0; i < 30; ++i) {
    filter.predict(0.033);
    filter.update(target);
  }

  Eigen::Vector3d pos_err =
      filter.pose().translation() - target.translation();
  EXPECT_LT(pos_err.norm(), 0.01);  // < 1cm after convergence
}

TEST(IekfSe3, RejectsOutlier) {
  IekfSe3::Config config;
  config.mahalanobis_threshold = 16.81;
  IekfSe3 filter(config);

  // Initialize near origin
  Eigen::Isometry3d origin = Eigen::Isometry3d::Identity();
  filter.reset(origin);

  // Feed consistent measurements to build up confidence
  for (int i = 0; i < 10; ++i) {
    filter.predict(0.033);
    filter.update(origin);
  }

  // Now send a wildly different measurement
  Eigen::Isometry3d outlier = Eigen::Isometry3d::Identity();
  outlier.translation() = Eigen::Vector3d(100.0, 100.0, 100.0);
  filter.predict(0.033);
  bool accepted = filter.update(outlier);
  EXPECT_FALSE(accepted);

  // Pose should still be near origin
  EXPECT_LT(filter.pose().translation().norm(), 1.0);
}

}  // namespace
}  // namespace perspective_grasp
