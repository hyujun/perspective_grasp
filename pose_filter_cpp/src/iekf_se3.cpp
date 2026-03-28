#include "pose_filter_cpp/iekf_se3.hpp"

#include <cmath>

namespace perspective_grasp {

namespace {

/// Skew-symmetric matrix from 3-vector
Eigen::Matrix3d skew(const Eigen::Vector3d& v) {
  Eigen::Matrix3d S;
  S << 0, -v.z(), v.y(),
       v.z(), 0, -v.x(),
       -v.y(), v.x(), 0;
  return S;
}

/// Small-angle rotation vector to rotation matrix (Rodrigues)
Eigen::Matrix3d expSO3(const Eigen::Vector3d& omega) {
  double theta = omega.norm();
  if (theta < 1e-10) {
    return Eigen::Matrix3d::Identity() + skew(omega);
  }
  Eigen::Vector3d axis = omega / theta;
  return Eigen::AngleAxisd(theta, axis).toRotationMatrix();
}

/// Rotation matrix to rotation vector (log map)
Eigen::Vector3d logSO3(const Eigen::Matrix3d& R) {
  Eigen::AngleAxisd aa(R);
  return aa.angle() * aa.axis();
}

}  // namespace

IekfSe3::IekfSe3() : IekfSe3(Config{}) {}

IekfSe3::IekfSe3(const Config& config) : config_(config) {
  P_ = Eigen::Matrix<double, 12, 12>::Identity() * 0.1;
}

void IekfSe3::reset(const Eigen::Isometry3d& pose) {
  pose_ = pose;
  twist_.setZero();
  P_ = Eigen::Matrix<double, 12, 12>::Identity() * 0.01;
  initialized_ = true;
}

void IekfSe3::predict(double dt) {
  if (!initialized_ || dt <= 0.0) return;

  // Extract twist components
  Eigen::Vector3d omega = twist_.head<3>();
  Eigen::Vector3d vel = twist_.tail<3>();

  // Integrate pose: T_new = T * exp(twist * dt)
  Eigen::Matrix3d dR = expSO3(omega * dt);
  Eigen::Vector3d dp = vel * dt;

  pose_.linear() = pose_.linear() * dR;
  pose_.translation() += pose_.linear() * dp;

  // State transition Jacobian F (12x12)
  Eigen::Matrix<double, 12, 12> F =
      Eigen::Matrix<double, 12, 12>::Identity();
  // d(rot)/d(omega) = I * dt
  F.block<3, 3>(0, 6) = Eigen::Matrix3d::Identity() * dt;
  // d(pos)/d(vel) = R * dt
  F.block<3, 3>(3, 9) = pose_.linear() * dt;

  // Process noise Q
  Eigen::Matrix<double, 12, 12> Q =
      Eigen::Matrix<double, 12, 12>::Zero();
  double dt2 = dt * dt;
  Q.block<3, 3>(0, 0) =
      Eigen::Matrix3d::Identity() * config_.process_noise_rot *
      config_.process_noise_rot * dt2;
  Q.block<3, 3>(3, 3) =
      Eigen::Matrix3d::Identity() * config_.process_noise_pos *
      config_.process_noise_pos * dt2;
  Q.block<3, 3>(6, 6) =
      Eigen::Matrix3d::Identity() * config_.process_noise_omega *
      config_.process_noise_omega * dt2;
  Q.block<3, 3>(9, 9) =
      Eigen::Matrix3d::Identity() * config_.process_noise_vel *
      config_.process_noise_vel * dt2;

  P_ = F * P_ * F.transpose() + Q;
}

Eigen::Matrix<double, 6, 1> IekfSe3::computeInnovation(
    const Eigen::Isometry3d& measurement) const {
  // Innovation y = log(T_meas * T_pred^{-1})
  Eigen::Isometry3d delta = measurement * pose_.inverse();
  Eigen::Matrix<double, 6, 1> y;
  y.head<3>() = logSO3(delta.linear());
  y.tail<3>() = delta.translation();
  return y;
}

bool IekfSe3::update(const Eigen::Isometry3d& measurement,
                      double noise_scale) {
  if (!initialized_) {
    reset(measurement);
    return true;
  }

  // Measurement noise R (6x6)
  Eigen::Matrix<double, 6, 6> R = Eigen::Matrix<double, 6, 6>::Zero();
  double sr = config_.meas_noise_rot * noise_scale;
  double sp = config_.meas_noise_pos * noise_scale;
  R.block<3, 3>(0, 0) = Eigen::Matrix3d::Identity() * sr * sr;
  R.block<3, 3>(3, 3) = Eigen::Matrix3d::Identity() * sp * sp;

  // Observation Jacobian H: measures pose (first 6 states)
  Eigen::Matrix<double, 6, 12> H = Eigen::Matrix<double, 6, 12>::Zero();
  H.block<6, 6>(0, 0) = Eigen::Matrix<double, 6, 6>::Identity();

  // Iterated update
  auto P_iter = P_;

  for (int iter = 0; iter < config_.max_iterations; ++iter) {
    Eigen::Matrix<double, 6, 1> y = computeInnovation(measurement);

    // Innovation covariance S
    Eigen::Matrix<double, 6, 6> S = H * P_iter * H.transpose() + R;

    // Mahalanobis distance for outlier rejection (first iteration only)
    if (iter == 0) {
      double mahal = y.transpose() * S.inverse() * y;
      if (mahal > config_.mahalanobis_threshold) {
        return false;  // Reject outlier
      }
    }

    // Kalman gain
    Eigen::Matrix<double, 12, 6> K = P_iter * H.transpose() * S.inverse();

    // State correction (12D)
    Eigen::Matrix<double, 12, 1> dx = K * y;

    // Apply correction to pose
    pose_.linear() = expSO3(dx.head<3>()) * pose_.linear();
    pose_.translation() += dx.segment<3>(3);

    // Apply correction to twist
    twist_ += dx.tail<6>();

    // Update covariance (Joseph form for numerical stability)
    auto I_KH = Eigen::Matrix<double, 12, 12>::Identity() - K * H;
    P_ = I_KH * P_iter * I_KH.transpose() + K * R * K.transpose();
    P_iter = P_;
  }

  return true;
}

Eigen::Matrix<double, 6, 6> IekfSe3::poseCovariance() const {
  return P_.block<6, 6>(0, 0);
}

}  // namespace perspective_grasp
