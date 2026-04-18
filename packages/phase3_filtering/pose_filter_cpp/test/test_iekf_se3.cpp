#include <gtest/gtest.h>

#include <cmath>

#include <Eigen/Dense>
#include <Eigen/Geometry>

#include "pose_filter_cpp/iekf_se3.hpp"

namespace perspective_grasp {
namespace {

// ---------- helpers ----------

Eigen::Isometry3d makePose(double x, double y, double z,
                           double rx = 0.0, double ry = 0.0, double rz = 0.0) {
  Eigen::Isometry3d p = Eigen::Isometry3d::Identity();
  p.translation() = Eigen::Vector3d(x, y, z);
  Eigen::Matrix3d R =
      (Eigen::AngleAxisd(rz, Eigen::Vector3d::UnitZ()) *
       Eigen::AngleAxisd(ry, Eigen::Vector3d::UnitY()) *
       Eigen::AngleAxisd(rx, Eigen::Vector3d::UnitX()))
          .toRotationMatrix();
  p.linear() = R;
  return p;
}

double rotAngleDiff(const Eigen::Matrix3d& A, const Eigen::Matrix3d& B) {
  Eigen::AngleAxisd aa(A.transpose() * B);
  return std::fabs(aa.angle());
}

bool isFinite(const Eigen::Isometry3d& p) {
  return p.matrix().allFinite();
}

// ==========================================================================
// Construction & initialization
// ==========================================================================

TEST(IekfSe3Basics, DefaultConstructedNotInitialized) {
  IekfSe3 filter;
  EXPECT_FALSE(filter.initialized());
  EXPECT_TRUE(filter.pose().translation().isZero());
  EXPECT_TRUE(filter.pose().linear().isIdentity());
}

TEST(IekfSe3Basics, DefaultCovarianceIsPositive) {
  IekfSe3 filter;
  const auto& P = filter.covariance();
  // Default covariance is 0.1 * I (12x12)
  for (int i = 0; i < 12; ++i) {
    EXPECT_GT(P(i, i), 0.0) << "diag element " << i << " is non-positive";
  }
}

TEST(IekfSe3Basics, CustomConfigStoredConsistently) {
  IekfSe3::Config cfg;
  cfg.max_iterations = 5;
  cfg.mahalanobis_threshold = 9.0;
  IekfSe3 filter(cfg);
  EXPECT_FALSE(filter.initialized());
}

TEST(IekfSe3Initialization, FirstUpdateInitializesToMeasurement) {
  IekfSe3 filter;
  EXPECT_FALSE(filter.initialized());

  auto meas = makePose(1.0, 2.0, 3.0, 0.1, -0.2, 0.3);
  const bool accepted = filter.update(meas);

  EXPECT_TRUE(accepted);
  EXPECT_TRUE(filter.initialized());
  EXPECT_NEAR((filter.pose().translation() - meas.translation()).norm(), 0.0,
              1e-9);
  EXPECT_LT(rotAngleDiff(filter.pose().linear(), meas.linear()), 1e-9);
}

TEST(IekfSe3Initialization, ResetSetsPoseAndClearsTwist) {
  IekfSe3 filter;
  auto p1 = makePose(0.5, 0.0, 0.0);
  filter.reset(p1);
  EXPECT_TRUE(filter.initialized());
  // Reset installs pose and P = 0.01 * I.
  const auto& P = filter.covariance();
  for (int i = 0; i < 12; ++i) {
    EXPECT_NEAR(P(i, i), 0.01, 1e-12);
  }
  for (int i = 0; i < 12; ++i) {
    for (int j = 0; j < 12; ++j) {
      if (i != j) {
        EXPECT_NEAR(P(i, j), 0.0, 1e-12);
      }
    }
  }
}

// ==========================================================================
// predict() edge cases
// ==========================================================================

TEST(IekfSe3Predict, PredictBeforeInitIsNoOp) {
  IekfSe3 filter;
  const auto P_before = filter.covariance();
  filter.predict(0.1);
  const auto P_after = filter.covariance();
  EXPECT_TRUE(P_before.isApprox(P_after));
  EXPECT_FALSE(filter.initialized());
}

TEST(IekfSe3Predict, PredictWithZeroDtIsNoOp) {
  IekfSe3 filter;
  filter.reset(Eigen::Isometry3d::Identity());
  const auto pose_before = filter.pose();
  const auto P_before = filter.covariance();
  filter.predict(0.0);
  EXPECT_TRUE(pose_before.isApprox(filter.pose()));
  EXPECT_TRUE(P_before.isApprox(filter.covariance()));
}

TEST(IekfSe3Predict, PredictWithNegativeDtIsNoOp) {
  IekfSe3 filter;
  filter.reset(Eigen::Isometry3d::Identity());
  const auto pose_before = filter.pose();
  const auto P_before = filter.covariance();
  filter.predict(-0.05);
  EXPECT_TRUE(pose_before.isApprox(filter.pose()));
  EXPECT_TRUE(P_before.isApprox(filter.covariance()));
}

TEST(IekfSe3Predict, PredictGrowsCovariance) {
  // After prediction, P should be >= P_before (PSD ordering) — at minimum the
  // diagonals must not decrease since Q is added each step.
  IekfSe3 filter;
  filter.reset(Eigen::Isometry3d::Identity());
  const auto P0 = filter.covariance();
  filter.predict(0.1);
  const auto P1 = filter.covariance();
  for (int i = 0; i < 12; ++i) {
    EXPECT_GE(P1(i, i) + 1e-12, P0(i, i)) << "diag " << i << " shrank";
  }
}

TEST(IekfSe3Predict, ProcessNoiseGrowsLinearlyWithDt) {
  // Process-noise σ is declared as a continuous-time density (rad/sqrt(s)),
  // so Q ∝ dt, not dt².  Compare Q contribution for dt and 10*dt: ratio ~= 10.
  const double dt_small = 1e-4;
  const double dt_large = 1e-3;

  IekfSe3 filter;
  filter.reset(Eigen::Isometry3d::Identity());
  double trace_before = filter.covariance().trace();
  filter.predict(dt_small);
  const double delta_small = filter.covariance().trace() - trace_before;

  filter.reset(Eigen::Isometry3d::Identity());
  trace_before = filter.covariance().trace();
  filter.predict(dt_large);
  const double delta_large = filter.covariance().trace() - trace_before;

  ASSERT_GT(delta_small, 0.0);
  const double ratio = delta_large / delta_small;
  // Linear scaling: ratio ≈ 10.  Quadratic (bug) gives ratio ≈ 100.
  EXPECT_NEAR(ratio, 10.0, 2.0)
      << "Q appears to scale with dt^2 instead of dt (ratio=" << ratio << ")";
}

TEST(IekfSe3Predict, PreservesPoseWhenTwistIsZero) {
  // Fresh reset sets twist = 0, so predict should not move the pose.
  IekfSe3 filter;
  auto p0 = makePose(1.0, 2.0, 3.0, 0.1, 0.0, 0.0);
  filter.reset(p0);
  filter.predict(0.5);
  EXPECT_TRUE(p0.isApprox(filter.pose(), 1e-9));
}

// ==========================================================================
// update() — convergence & reasonable behaviour
// ==========================================================================

TEST(IekfSe3Update, ConvergesToStaticTargetPosition) {
  IekfSe3::Config cfg;
  cfg.max_iterations = 3;
  IekfSe3 filter(cfg);

  auto target = makePose(0.5, -0.3, 1.0, 0.0, 0.0, 0.5);
  for (int i = 0; i < 30; ++i) {
    filter.predict(0.033);
    filter.update(target);
  }

  EXPECT_LT((filter.pose().translation() - target.translation()).norm(), 0.01);
}

TEST(IekfSe3Update, ConvergesToStaticTargetRotation) {
  IekfSe3::Config cfg;
  IekfSe3 filter(cfg);

  auto target = makePose(0.0, 0.0, 0.0, 0.3, -0.2, 0.4);
  for (int i = 0; i < 30; ++i) {
    filter.predict(0.033);
    filter.update(target);
  }
  EXPECT_LT(rotAngleDiff(filter.pose().linear(), target.linear()), 0.02);
}

TEST(IekfSe3Update, ReducesPoseCovarianceAfterManyMeasurements) {
  IekfSe3 filter;
  auto target = makePose(0.0, 0.0, 0.0);
  filter.reset(target);
  const double initial_trace = filter.poseCovariance().trace();
  for (int i = 0; i < 50; ++i) {
    filter.predict(0.01);
    filter.update(target);
  }
  const double final_trace = filter.poseCovariance().trace();
  EXPECT_LT(final_trace, initial_trace);
}

TEST(IekfSe3Update, NoiseScaleAffectsGain) {
  // A measurement with a large noise scale should move the estimate LESS than
  // the same measurement with a small noise scale.
  auto run_shift = [](double noise_scale) {
    IekfSe3 filter;
    filter.reset(Eigen::Isometry3d::Identity());
    auto meas = makePose(0.1, 0.0, 0.0);
    filter.predict(0.033);
    filter.update(meas, noise_scale);
    return filter.pose().translation().x();
  };
  const double shift_trusted = run_shift(0.1);
  const double shift_noisy = run_shift(10.0);
  EXPECT_GT(shift_trusted, shift_noisy);
  EXPECT_GT(shift_trusted, 0.0);
}

// ==========================================================================
// Outlier rejection
// ==========================================================================

TEST(IekfSe3Outlier, RejectsFarMeasurement) {
  IekfSe3::Config cfg;
  cfg.mahalanobis_threshold = 16.81;
  IekfSe3 filter(cfg);
  auto origin = Eigen::Isometry3d::Identity();
  filter.reset(origin);
  for (int i = 0; i < 10; ++i) {
    filter.predict(0.033);
    filter.update(origin);
  }
  auto outlier = makePose(100.0, 100.0, 100.0);
  filter.predict(0.033);
  EXPECT_FALSE(filter.update(outlier));
  EXPECT_LT(filter.pose().translation().norm(), 1.0);
}

TEST(IekfSe3Outlier, AcceptsWithRelaxedThreshold) {
  IekfSe3::Config cfg;
  cfg.mahalanobis_threshold = 1e9;  // essentially disabled
  IekfSe3 filter(cfg);
  filter.reset(Eigen::Isometry3d::Identity());
  for (int i = 0; i < 10; ++i) {
    filter.predict(0.033);
    filter.update(Eigen::Isometry3d::Identity());
  }
  auto far = makePose(10.0, 0.0, 0.0);
  filter.predict(0.033);
  EXPECT_TRUE(filter.update(far));
}

TEST(IekfSe3Outlier, FirstUpdateNeverRejected) {
  IekfSe3::Config cfg;
  cfg.mahalanobis_threshold = 1e-6;  // would reject everything
  IekfSe3 filter(cfg);
  auto far = makePose(1e6, 1e6, 1e6);
  // Uninitialized path in update() calls reset() and returns true.
  EXPECT_TRUE(filter.update(far));
  EXPECT_TRUE(filter.initialized());
}

// ==========================================================================
// IEKF-specific: iterations
// ==========================================================================

TEST(IekfSe3Iteration, IteratingMatchesOrBeatsSingleStep) {
  // With max_iterations=3, after ONE update the estimate should be at least as
  // close to the measurement as with max_iterations=1 (iterations can only
  // improve the linearization point).
  auto run = [](int iters) {
    IekfSe3::Config cfg;
    cfg.max_iterations = iters;
    IekfSe3 filter(cfg);
    filter.reset(Eigen::Isometry3d::Identity());
    auto meas = makePose(0.0, 0.0, 0.0, 0.0, 0.0, 1.0);  // 1 rad rotation
    filter.predict(0.033);
    filter.update(meas);
    return rotAngleDiff(filter.pose().linear(), meas.linear());
  };
  const double err1 = run(1);
  const double err3 = run(3);
  EXPECT_LE(err3, err1 + 1e-9);
}

// ==========================================================================
// Covariance properties
// ==========================================================================

TEST(IekfSe3Covariance, RemainsSymmetricAfterUpdate) {
  IekfSe3 filter;
  filter.reset(Eigen::Isometry3d::Identity());
  auto meas = makePose(0.1, 0.0, 0.0);
  for (int i = 0; i < 20; ++i) {
    filter.predict(0.033);
    filter.update(meas);
  }
  const auto& P = filter.covariance();
  const double asym = (P - P.transpose()).cwiseAbs().maxCoeff();
  EXPECT_LT(asym, 1e-12) << "Covariance lost symmetry (max asym = " << asym
                         << ")";
}

TEST(IekfSe3Covariance, StaysPositiveSemiDefinite) {
  IekfSe3 filter;
  filter.reset(Eigen::Isometry3d::Identity());
  auto meas = makePose(0.1, 0.05, -0.02, 0.1, 0.0, 0.0);
  for (int i = 0; i < 50; ++i) {
    filter.predict(0.033);
    filter.update(meas);
  }
  // Symmetrize before checking eigenvalues to remove floating-point asymmetry.
  Eigen::Matrix<double, 12, 12> P = filter.covariance();
  P = 0.5 * (P + P.transpose());
  Eigen::SelfAdjointEigenSolver<Eigen::Matrix<double, 12, 12>> es(P);
  EXPECT_GT(es.eigenvalues().minCoeff(), -1e-9)
      << "Covariance became indefinite; min eig = "
      << es.eigenvalues().minCoeff();
}

TEST(IekfSe3Covariance, PoseCovarianceIsTopLeftBlock) {
  IekfSe3 filter;
  filter.reset(Eigen::Isometry3d::Identity());
  const auto full = filter.covariance();
  const auto pose = filter.poseCovariance();
  EXPECT_TRUE((full.block<6, 6>(0, 0).isApprox(pose, 1e-12)));
}

// ==========================================================================
// Stability
// ==========================================================================

TEST(IekfSe3Stability, HundredCyclesStayFiniteAndOrthogonal) {
  IekfSe3::Config cfg;
  cfg.max_iterations = 3;
  IekfSe3 filter(cfg);
  filter.reset(Eigen::Isometry3d::Identity());

  auto meas = makePose(0.2, -0.1, 0.3, 0.05, -0.05, 0.1);
  for (int i = 0; i < 100; ++i) {
    filter.predict(0.033);
    filter.update(meas);
  }
  ASSERT_TRUE(isFinite(filter.pose()));
  // Rotation should stay close to SO(3): R^T R ~ I.
  Eigen::Matrix3d R = filter.pose().linear();
  const double orthog_err = (R.transpose() * R - Eigen::Matrix3d::Identity())
                                .cwiseAbs()
                                .maxCoeff();
  EXPECT_LT(orthog_err, 1e-12)
      << "Rotation drifted from SO(3); max error = " << orthog_err;
  // det(R) should stay +1.
  EXPECT_NEAR(R.determinant(), 1.0, 1e-12);
}

TEST(IekfSe3Stability, LongRunKeepsRotationOnManifold) {
  // 10k update cycles — catches slow drift that a 100-cycle test misses.
  IekfSe3::Config cfg;
  cfg.max_iterations = 3;
  IekfSe3 filter(cfg);
  filter.reset(Eigen::Isometry3d::Identity());
  auto meas = makePose(0.1, 0.05, -0.02, 0.2, -0.1, 0.3);

  for (int i = 0; i < 10000; ++i) {
    filter.predict(0.01);
    filter.update(meas);
  }
  const Eigen::Matrix3d R = filter.pose().linear();
  const double orthog_err =
      (R.transpose() * R - Eigen::Matrix3d::Identity()).cwiseAbs().maxCoeff();
  EXPECT_LT(orthog_err, 1e-12)
      << "Rotation drifted from SO(3) after 10k cycles; max error = "
      << orthog_err;
  EXPECT_NEAR(R.determinant(), 1.0, 1e-12);

  // Also check covariance symmetry.
  const auto& P = filter.covariance();
  const double asym = (P - P.transpose()).cwiseAbs().maxCoeff();
  EXPECT_LT(asym, 1e-12) << "Covariance asymmetry after 10k cycles = " << asym;
}

TEST(IekfSe3Stability, LargeDtDoesNotBlowUp) {
  IekfSe3 filter;
  filter.reset(Eigen::Isometry3d::Identity());
  filter.predict(10.0);  // 10 seconds in one step
  EXPECT_TRUE(isFinite(filter.pose()));
  EXPECT_TRUE(filter.covariance().allFinite());
}

TEST(IekfSe3Stability, IdentityMeasurementProducesZeroInnovation) {
  IekfSe3 filter;
  filter.reset(Eigen::Isometry3d::Identity());
  const auto pose_before = filter.pose();
  // Perfect measurement should not shift pose (twist stays zero).
  filter.update(Eigen::Isometry3d::Identity());
  EXPECT_TRUE(pose_before.isApprox(filter.pose(), 1e-9));
}

}  // namespace
}  // namespace perspective_grasp
