// Copyright 2026 perspective_grasp
// SPDX-License-Identifier: Apache-2.0

#include "multi_camera_calibration/hand_eye_solver.hpp"

#include "multi_camera_calibration/detail/pose_converters.hpp"

#include <opencv2/calib3d.hpp>
#include <opencv2/core.hpp>

#include <rclcpp/rclcpp.hpp>

#include <stdexcept>

namespace perspective_grasp {

using detail::isometryToRvecTvec;
using detail::rvecTvecToIsometry;

CalibrationResult HandEyeSolver::solveEyeInHand(
    const std::vector<Eigen::Isometry3d>& T_base_gripper,
    const std::vector<Eigen::Isometry3d>& T_target_cam) const {
  CalibrationResult result;

  if (T_base_gripper.size() != T_target_cam.size()) {
    RCLCPP_ERROR(rclcpp::get_logger("hand_eye_solver"),
                 "Mismatched sample sizes: %zu vs %zu", T_base_gripper.size(),
                 T_target_cam.size());
    return result;
  }

  if (T_base_gripper.size() < 3) {
    RCLCPP_ERROR(rclcpp::get_logger("hand_eye_solver"),
                 "Need at least 3 samples, got %zu", T_base_gripper.size());
    return result;
  }

  // Convert to OpenCV format: vectors of rotation and translation matrices
  std::vector<cv::Mat> R_gripper2base, t_gripper2base;
  std::vector<cv::Mat> R_target2cam, t_target2cam;

  for (size_t i = 0; i < T_base_gripper.size(); ++i) {
    cv::Mat rvec_bg, tvec_bg;
    isometryToRvecTvec(T_base_gripper[i], rvec_bg, tvec_bg);

    // calibrateHandEye expects R_gripper2base and t_gripper2base
    // T_base_gripper is already in this form (base->gripper), but OpenCV
    // convention uses rotation matrices (not rvec)
    cv::Mat R_bg;
    cv::Rodrigues(rvec_bg, R_bg);
    R_gripper2base.push_back(R_bg);
    t_gripper2base.push_back(tvec_bg);

    cv::Mat rvec_tc, tvec_tc;
    isometryToRvecTvec(T_target_cam[i], rvec_tc, tvec_tc);
    cv::Mat R_tc;
    cv::Rodrigues(rvec_tc, R_tc);
    R_target2cam.push_back(R_tc);
    t_target2cam.push_back(tvec_tc);
  }

  cv::Mat R_cam2gripper, t_cam2gripper;

  try {
    cv::calibrateHandEye(R_gripper2base, t_gripper2base, R_target2cam,
                         t_target2cam, R_cam2gripper, t_cam2gripper,
                         cv::CALIB_HAND_EYE_TSAI);
  } catch (const cv::Exception& e) {
    RCLCPP_ERROR(rclcpp::get_logger("hand_eye_solver"),
                 "calibrateHandEye failed: %s", e.what());
    return result;
  }

  // Convert result: T_cam_gripper -> for eye-in-hand, the result is
  // the camera-to-gripper transform. We store T_cam_base as the
  // camera-to-base relationship (T_cam_gripper here, user applies
  // T_base_gripper to get full chain).
  cv::Mat rvec_result;
  cv::Rodrigues(R_cam2gripper, rvec_result);
  result.T_cam_base = rvecTvecToIsometry(rvec_result, t_cam2gripper);
  result.success = true;
  result.reprojection_error = 0.0;  // OpenCV does not return this directly

  RCLCPP_INFO(rclcpp::get_logger("hand_eye_solver"),
              "Eye-in-hand calibration succeeded with %zu samples",
              T_base_gripper.size());

  return result;
}

CalibrationResult HandEyeSolver::solveEyeToHand(
    const std::vector<Eigen::Isometry3d>& T_gripper_base,
    const std::vector<Eigen::Isometry3d>& T_target_cam) const {
  CalibrationResult result;

  if (T_gripper_base.size() != T_target_cam.size()) {
    RCLCPP_ERROR(rclcpp::get_logger("hand_eye_solver"),
                 "Mismatched sample sizes: %zu vs %zu", T_gripper_base.size(),
                 T_target_cam.size());
    return result;
  }

  if (T_gripper_base.size() < 3) {
    RCLCPP_ERROR(rclcpp::get_logger("hand_eye_solver"),
                 "Need at least 3 samples, got %zu", T_gripper_base.size());
    return result;
  }

  // For eye-to-hand: camera is fixed, target is on the gripper.
  // OpenCV calibrateHandEye solves AX=XB where:
  //   A = T_gripper_base (robot motion)
  //   B = T_target_cam (observed target motion)
  // Result X = T_cam_base (camera to base transform)
  std::vector<cv::Mat> R_base2gripper, t_base2gripper;
  std::vector<cv::Mat> R_target2cam, t_target2cam;

  for (size_t i = 0; i < T_gripper_base.size(); ++i) {
    cv::Mat rvec_gb, tvec_gb;
    isometryToRvecTvec(T_gripper_base[i], rvec_gb, tvec_gb);
    cv::Mat R_gb;
    cv::Rodrigues(rvec_gb, R_gb);
    R_base2gripper.push_back(R_gb);
    t_base2gripper.push_back(tvec_gb);

    cv::Mat rvec_tc, tvec_tc;
    isometryToRvecTvec(T_target_cam[i], rvec_tc, tvec_tc);
    cv::Mat R_tc;
    cv::Rodrigues(rvec_tc, R_tc);
    R_target2cam.push_back(R_tc);
    t_target2cam.push_back(tvec_tc);
  }

  cv::Mat R_cam2base, t_cam2base;

  try {
    cv::calibrateHandEye(R_base2gripper, t_base2gripper, R_target2cam,
                         t_target2cam, R_cam2base, t_cam2base,
                         cv::CALIB_HAND_EYE_TSAI);
  } catch (const cv::Exception& e) {
    RCLCPP_ERROR(rclcpp::get_logger("hand_eye_solver"),
                 "calibrateHandEye failed: %s", e.what());
    return result;
  }

  cv::Mat rvec_result;
  cv::Rodrigues(R_cam2base, rvec_result);
  result.T_cam_base = rvecTvecToIsometry(rvec_result, t_cam2base);
  result.success = true;
  result.reprojection_error = 0.0;

  RCLCPP_INFO(rclcpp::get_logger("hand_eye_solver"),
              "Eye-to-hand calibration succeeded with %zu samples",
              T_gripper_base.size());

  return result;
}

}  // namespace perspective_grasp
