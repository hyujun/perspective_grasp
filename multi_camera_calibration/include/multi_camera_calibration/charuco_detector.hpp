// Copyright 2026 perspective_grasp
// SPDX-License-Identifier: Apache-2.0

#pragma once

#include <opencv2/aruco.hpp>
#include <opencv2/aruco/charuco.hpp>
#include <opencv2/core.hpp>

#include <string>
#include <vector>

namespace perspective_grasp {

/// Configuration for ChArUco board detection.
struct CharucoBoardConfig {
  int squares_x = 7;
  int squares_y = 5;
  float square_length = 0.04F;
  float marker_length = 0.03F;
  std::string dictionary_name = "DICT_6X6_250";
  int min_corners = 4;
};

/// Result of a single ChArUco board detection.
struct DetectionResult {
  bool success = false;
  cv::Vec3d rvec{};
  cv::Vec3d tvec{};
  std::vector<cv::Point2f> corners_2d;
  std::vector<cv::Point3f> corners_3d;
  double reprojection_error = 0.0;
};

/// Detects a ChArUco calibration board in images.
class CharucoDetector {
 public:
  /// Construct a detector with the given board configuration.
  explicit CharucoDetector(const CharucoBoardConfig& config);

  /// Detect the ChArUco board in an image and estimate its pose.
  /// @param image Input BGR image.
  /// @param camera_matrix 3x3 camera intrinsic matrix.
  /// @param dist_coeffs Distortion coefficients.
  /// @return Detection result with pose and corners.
  [[nodiscard]] DetectionResult detect(const cv::Mat& image,
                                       const cv::Mat& camera_matrix,
                                       const cv::Mat& dist_coeffs) const;

  /// Get the underlying ChArUco board.
  [[nodiscard]] cv::Ptr<cv::aruco::CharucoBoard> getBoard() const {
    return board_;
  }

 private:
  CharucoBoardConfig config_;
  cv::Ptr<cv::aruco::Dictionary> dictionary_;
  cv::Ptr<cv::aruco::CharucoBoard> board_;
  cv::Ptr<cv::aruco::DetectorParameters> detector_params_;

  /// Map dictionary name string to OpenCV ArUco predefined dictionary.
  static cv::aruco::PREDEFINED_DICTIONARY_NAME parseDictionary(
      const std::string& name);
};

}  // namespace perspective_grasp
