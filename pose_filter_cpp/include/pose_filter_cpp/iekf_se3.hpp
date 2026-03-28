#pragma once

#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <optional>

namespace perspective_grasp {

/// Lightweight SE(3) EKF using Eigen only.
/// State: pose (Isometry3d) + twist (6D body velocity).
/// Covariance lives in the 12D tangent space [rot(3) + trans(3) + omega(3) + vel(3)].
/// When manif is installed, this can be upgraded to a proper IEKF on Lie groups.
class IekfSe3 {
 public:
  struct Config {
    // Process noise standard deviations
    double process_noise_rot{0.01};    // rad/sqrt(s)
    double process_noise_pos{0.001};   // m/sqrt(s)
    double process_noise_omega{0.1};   // rad/s/sqrt(s)
    double process_noise_vel{0.1};     // m/s/sqrt(s)

    // Measurement noise (default, overridden per source)
    double meas_noise_rot{0.05};       // rad
    double meas_noise_pos{0.005};      // m

    // IEKF iteration count (1 = standard EKF)
    int max_iterations{3};

    // Outlier rejection (Mahalanobis distance squared threshold, chi2(6, 0.01))
    double mahalanobis_threshold{16.81};
  };

  IekfSe3();
  explicit IekfSe3(const Config& config);

  /// Predict step using constant-velocity model
  void predict(double dt);

  /// Update with a new pose measurement.
  /// @param measurement The measured SE(3) pose
  /// @param noise_scale Multiplier on default measurement noise (lower = more trusted)
  /// @return false if measurement rejected as outlier
  bool update(const Eigen::Isometry3d& measurement, double noise_scale = 1.0);

  /// Get current estimated pose
  Eigen::Isometry3d pose() const { return pose_; }

  /// Get current 6x6 pose covariance (rotation + translation in tangent space)
  Eigen::Matrix<double, 6, 6> poseCovariance() const;

  /// Get full 12x12 state covariance
  const Eigen::Matrix<double, 12, 12>& covariance() const { return P_; }

  /// Reset filter to a given pose
  void reset(const Eigen::Isometry3d& pose);

  /// Check if filter has been initialized
  bool initialized() const { return initialized_; }

 private:
  /// Compute the 6D innovation: log(measurement * pose_^{-1}) approximation
  Eigen::Matrix<double, 6, 1> computeInnovation(
      const Eigen::Isometry3d& measurement) const;

  Config config_;
  Eigen::Isometry3d pose_{Eigen::Isometry3d::Identity()};
  Eigen::Matrix<double, 6, 1> twist_{Eigen::Matrix<double, 6, 1>::Zero()};  // [omega, vel]
  Eigen::Matrix<double, 12, 12> P_;  // State covariance
  bool initialized_{false};
};

}  // namespace perspective_grasp
