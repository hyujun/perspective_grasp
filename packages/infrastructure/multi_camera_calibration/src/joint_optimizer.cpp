// Copyright 2026 perspective_grasp
// SPDX-License-Identifier: Apache-2.0

#include "multi_camera_calibration/joint_optimizer.hpp"

#ifdef HAS_CERES
#include <ceres/ceres.h>
#include <ceres/rotation.h>
#endif

#include <rclcpp/rclcpp.hpp>

namespace perspective_grasp {

#ifdef HAS_CERES

namespace {

/// Represents a 6-DoF pose as [angle_axis(3), translation(3)] for Ceres.
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

/// Reprojection error cost function for a single observation.
struct ReprojectionCost {
  Eigen::Vector3d point_3d;
  Eigen::Vector2d observed_2d;
  double fx, fy, cx, cy;

  ReprojectionCost(const Eigen::Vector3d& p3d, const Eigen::Vector2d& p2d,
                   double fx, double fy, double cx, double cy)
      : point_3d(p3d), observed_2d(p2d), fx(fx), fy(fy), cx(cx), cy(cy) {}

  template <typename T>
  bool operator()(const T* const cam_pose, const T* const board_pose,
                  const T* const robot_pose_inv, T* residual) const {
    // Chain: p_cam = T_cam_base * T_base_gripper * T_board_ee * p_board
    // robot_pose_inv represents T_base_gripper (given, fixed per sample)
    // cam_pose represents T_cam_base (optimized per camera)
    // board_pose represents T_board_ee (optimized, shared)

    // Transform point by board_pose (T_board_ee)
    T p_board[3] = {T(point_3d.x()), T(point_3d.y()), T(point_3d.z())};
    T p_ee[3];
    ceres::AngleAxisRotatePoint(board_pose, p_board, p_ee);
    p_ee[0] += board_pose[3];
    p_ee[1] += board_pose[4];
    p_ee[2] += board_pose[5];

    // Transform by robot_pose (T_base_gripper)
    T p_base[3];
    ceres::AngleAxisRotatePoint(robot_pose_inv, p_ee, p_base);
    p_base[0] += robot_pose_inv[3];
    p_base[1] += robot_pose_inv[4];
    p_base[2] += robot_pose_inv[5];

    // Transform by cam_pose (T_cam_base)
    T p_cam[3];
    ceres::AngleAxisRotatePoint(cam_pose, p_base, p_cam);
    p_cam[0] += cam_pose[3];
    p_cam[1] += cam_pose[4];
    p_cam[2] += cam_pose[5];

    // Project
    T projected_x = T(fx) * p_cam[0] / p_cam[2] + T(cx);
    T projected_y = T(fy) * p_cam[1] / p_cam[2] + T(cy);

    residual[0] = projected_x - T(observed_2d.x());
    residual[1] = projected_y - T(observed_2d.y());

    return true;
  }

  static ceres::CostFunction* create(const Eigen::Vector3d& p3d,
                                     const Eigen::Vector2d& p2d, double fx,
                                     double fy, double cx, double cy) {
    return new ceres::AutoDiffCostFunction<ReprojectionCost, 2, 6, 6, 6>(
        new ReprojectionCost(p3d, p2d, fx, fy, cx, cy));
  }
};

}  // namespace

bool JointOptimizer::isAvailable() { return true; }

JointResult JointOptimizer::optimize(
    const std::vector<CalibrationSample>& samples,
    const std::map<int, CameraIntrinsics>& intrinsics,
    int num_cameras) const {
  JointResult result;

  if (samples.empty()) {
    result.message = "No calibration samples provided";
    return result;
  }

  auto logger = rclcpp::get_logger("joint_optimizer");
  RCLCPP_INFO(logger, "Starting joint optimization with %zu samples, %d cameras",
              samples.size(), num_cameras);

  // Initialize parameter blocks
  // One Pose6D per camera (T_cam_base), one shared T_board_ee
  std::map<int, Pose6D> cam_poses;
  for (int i = 0; i < num_cameras; ++i) {
    cam_poses[i] = Pose6D{};  // Identity initialization
  }
  Pose6D board_pose{};  // T_board_ee initialization

  // Per-sample robot poses (fixed, not optimized)
  std::vector<Pose6D> robot_poses;
  robot_poses.reserve(samples.size());
  for (const auto& sample : samples) {
    robot_poses.push_back(Pose6D::fromIsometry(sample.T_base_gripper));
  }

  // Build the Ceres problem
  ceres::Problem problem;

  for (size_t s = 0; s < samples.size(); ++s) {
    const auto& sample = samples[s];
    auto it = intrinsics.find(sample.camera_id);
    if (it == intrinsics.end()) {
      RCLCPP_WARN(logger, "No intrinsics for camera %d, skipping sample",
                  sample.camera_id);
      continue;
    }

    const auto& intr = it->second;
    size_t num_points = std::min(sample.points_3d.size(), sample.points_2d.size());

    for (size_t p = 0; p < num_points; ++p) {
      ceres::CostFunction* cost = ReprojectionCost::create(
          sample.points_3d[p], sample.points_2d[p], intr.fx, intr.fy, intr.cx,
          intr.cy);

      problem.AddResidualBlock(cost, new ceres::HuberLoss(1.0),
                               cam_poses[sample.camera_id].data,
                               board_pose.data, robot_poses[s].data);

      // Robot poses are constant (not optimized)
      problem.SetParameterBlockConstant(robot_poses[s].data);
    }
  }

  // Solve
  ceres::Solver::Options options;
  options.linear_solver_type = ceres::SPARSE_NORMAL_CHOLESKY;
  options.max_num_iterations = 200;
  options.function_tolerance = 1e-8;
  options.parameter_tolerance = 1e-8;
  options.minimizer_progress_to_stdout = true;

  ceres::Solver::Summary summary;
  ceres::Solve(options, &problem, &summary);

  RCLCPP_INFO(logger, "Optimization summary:\n%s",
              summary.BriefReport().c_str());

  // Extract results
  for (int i = 0; i < num_cameras; ++i) {
    result.T_cam_base[i] = cam_poses[i].toIsometry();
  }
  result.T_board_ee = board_pose.toIsometry();
  result.final_error = summary.final_cost;
  result.converged =
      (summary.termination_type == ceres::CONVERGENCE);
  result.message = summary.BriefReport();

  return result;
}

#else  // !HAS_CERES

bool JointOptimizer::isAvailable() { return false; }

JointResult JointOptimizer::optimize(
    const std::vector<CalibrationSample>& /*samples*/,
    const std::map<int, CameraIntrinsics>& /*intrinsics*/,
    int /*num_cameras*/) const {
  JointResult result;
  result.converged = false;
  result.message = "Ceres not available - install libceres-dev for joint optimization";

  RCLCPP_WARN(rclcpp::get_logger("joint_optimizer"), "%s",
              result.message.c_str());

  return result;
}

#endif  // HAS_CERES

}  // namespace perspective_grasp
