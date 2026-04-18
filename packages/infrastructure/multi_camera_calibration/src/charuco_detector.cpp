// Copyright 2026 perspective_grasp
// SPDX-License-Identifier: Apache-2.0

#include "multi_camera_calibration/charuco_detector.hpp"

#include <opencv2/calib3d.hpp>
#include <opencv2/imgproc.hpp>

#include <stdexcept>

namespace perspective_grasp {

CharucoDetector::CharucoDetector(const CharucoBoardConfig& config)
    : config_(config) {
  dictionary_ =
      cv::aruco::getPredefinedDictionary(parseDictionary(config_.dictionary_name));

  board_ = cv::aruco::CharucoBoard::create(
      config_.squares_x, config_.squares_y, config_.square_length,
      config_.marker_length, dictionary_);

  detector_params_ = cv::aruco::DetectorParameters::create();
  // Tune for better corner accuracy
  detector_params_->cornerRefinementMethod =
      cv::aruco::CORNER_REFINE_SUBPIX;
}

DetectionResult CharucoDetector::detect(const cv::Mat& image,
                                        const cv::Mat& camera_matrix,
                                        const cv::Mat& dist_coeffs) const {
  DetectionResult result;

  if (image.empty()) {
    return result;
  }

  // Convert to grayscale if needed
  cv::Mat gray;
  if (image.channels() == 3) {
    cv::cvtColor(image, gray, cv::COLOR_BGR2GRAY);
  } else {
    gray = image;
  }

  // Step 1: Detect ArUco markers
  std::vector<int> marker_ids;
  std::vector<std::vector<cv::Point2f>> marker_corners;
  std::vector<std::vector<cv::Point2f>> rejected_candidates;

  cv::aruco::detectMarkers(gray, dictionary_, marker_corners, marker_ids,
                           detector_params_, rejected_candidates);

  if (marker_ids.empty()) {
    return result;
  }

  // Step 2: Refine detected markers
  cv::aruco::refineDetectedMarkers(gray, board_, marker_corners, marker_ids,
                                   rejected_candidates, camera_matrix,
                                   dist_coeffs);

  // Step 3: Interpolate ChArUco corners
  std::vector<cv::Point2f> charuco_corners;
  std::vector<int> charuco_ids;

  int num_corners = cv::aruco::interpolateCornersCharuco(
      marker_corners, marker_ids, gray, board_, charuco_corners, charuco_ids,
      camera_matrix, dist_coeffs);

  if (num_corners < config_.min_corners) {
    return result;
  }

  // Step 4: Estimate board pose using solvePnP
  cv::Vec3d rvec, tvec;
  bool valid = cv::aruco::estimatePoseCharucoBoard(
      charuco_corners, charuco_ids, board_, camera_matrix, dist_coeffs, rvec,
      tvec);

  if (!valid) {
    return result;
  }

  // Step 5: Compute reprojection error
  // Get the 3D object points for the detected charuco corners
  std::vector<cv::Point3f> object_points;
  const auto& all_charuco_corners_3d = board_->chessboardCorners;
  for (int id : charuco_ids) {
    object_points.push_back(all_charuco_corners_3d[static_cast<size_t>(id)]);
  }

  std::vector<cv::Point2f> reprojected;
  cv::projectPoints(object_points, rvec, tvec, camera_matrix, dist_coeffs,
                    reprojected);

  double total_error = 0.0;
  for (size_t i = 0; i < charuco_corners.size(); ++i) {
    double dx = charuco_corners[i].x - reprojected[i].x;
    double dy = charuco_corners[i].y - reprojected[i].y;
    total_error += std::sqrt(dx * dx + dy * dy);
  }
  double mean_error = total_error / static_cast<double>(charuco_corners.size());

  // Fill result
  result.success = true;
  result.rvec = rvec;
  result.tvec = tvec;
  result.corners_2d = charuco_corners;
  result.corners_3d = object_points;
  result.reprojection_error = mean_error;

  return result;
}

cv::aruco::PREDEFINED_DICTIONARY_NAME CharucoDetector::parseDictionary(
    const std::string& name) {
  if (name == "DICT_4X4_50") return cv::aruco::DICT_4X4_50;
  if (name == "DICT_4X4_100") return cv::aruco::DICT_4X4_100;
  if (name == "DICT_4X4_250") return cv::aruco::DICT_4X4_250;
  if (name == "DICT_4X4_1000") return cv::aruco::DICT_4X4_1000;
  if (name == "DICT_5X5_50") return cv::aruco::DICT_5X5_50;
  if (name == "DICT_5X5_100") return cv::aruco::DICT_5X5_100;
  if (name == "DICT_5X5_250") return cv::aruco::DICT_5X5_250;
  if (name == "DICT_5X5_1000") return cv::aruco::DICT_5X5_1000;
  if (name == "DICT_6X6_50") return cv::aruco::DICT_6X6_50;
  if (name == "DICT_6X6_100") return cv::aruco::DICT_6X6_100;
  if (name == "DICT_6X6_250") return cv::aruco::DICT_6X6_250;
  if (name == "DICT_6X6_1000") return cv::aruco::DICT_6X6_1000;
  if (name == "DICT_7X7_50") return cv::aruco::DICT_7X7_50;
  if (name == "DICT_7X7_100") return cv::aruco::DICT_7X7_100;
  if (name == "DICT_7X7_250") return cv::aruco::DICT_7X7_250;
  if (name == "DICT_7X7_1000") return cv::aruco::DICT_7X7_1000;
  if (name == "DICT_ARUCO_ORIGINAL") return cv::aruco::DICT_ARUCO_ORIGINAL;

  throw std::invalid_argument("Unknown ArUco dictionary: " + name);
}

}  // namespace perspective_grasp
