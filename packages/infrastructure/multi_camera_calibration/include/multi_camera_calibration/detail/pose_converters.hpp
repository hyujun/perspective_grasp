// Copyright 2026 perspective_grasp
// SPDX-License-Identifier: Apache-2.0

#pragma once

#include <Eigen/Geometry>
#include <opencv2/core.hpp>
#include <opencv2/core/eigen.hpp>
#include <opencv2/calib3d.hpp>

namespace perspective_grasp::detail {

/// Convert an Eigen SE(3) transform into an OpenCV (rvec, tvec) pair
/// using Rodrigues representation.
inline void isometryToRvecTvec(const Eigen::Isometry3d& transform,
                               cv::Mat& rvec, cv::Mat& tvec) {
  Eigen::Matrix3d rotation = transform.rotation();
  Eigen::Vector3d translation = transform.translation();

  cv::Mat rotation_mat(3, 3, CV_64F);
  cv::eigen2cv(rotation, rotation_mat);
  cv::Rodrigues(rotation_mat, rvec);

  tvec = cv::Mat(3, 1, CV_64F);
  tvec.at<double>(0) = translation.x();
  tvec.at<double>(1) = translation.y();
  tvec.at<double>(2) = translation.z();
}

/// Convert an OpenCV (rvec, tvec) pair back into an Eigen SE(3) transform.
inline Eigen::Isometry3d rvecTvecToIsometry(const cv::Mat& rvec,
                                            const cv::Mat& tvec) {
  cv::Mat rotation_mat;
  cv::Rodrigues(rvec, rotation_mat);

  Eigen::Matrix3d rotation;
  cv::cv2eigen(rotation_mat, rotation);

  Eigen::Isometry3d result = Eigen::Isometry3d::Identity();
  result.linear() = rotation;
  result.translation() = Eigen::Vector3d(
      tvec.at<double>(0), tvec.at<double>(1), tvec.at<double>(2));

  return result;
}

}  // namespace perspective_grasp::detail
