// Copyright 2026 perspective_grasp
// SPDX-License-Identifier: Apache-2.0

#include "multi_camera_calibration/charuco_detector.hpp"

#include <gtest/gtest.h>

#include <opencv2/aruco/charuco.hpp>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>

#include <cmath>
#include <stdexcept>

namespace {

using perspective_grasp::CharucoBoardConfig;
using perspective_grasp::CharucoDetector;
using perspective_grasp::DetectionResult;

// Render the full board to a synthetic image and build intrinsics that
// correspond to a head-on camera observation of that rendering.
struct SyntheticBoardImage {
  cv::Mat image;
  cv::Mat camera_matrix;
  cv::Mat dist_coeffs;
};

SyntheticBoardImage renderBoardHeadOn(const CharucoDetector& detector,
                                      cv::Size image_size,
                                      int margin_px) {
  SyntheticBoardImage out;
  detector.getBoard()->draw(image_size, out.image, margin_px, 1);

  // Intrinsics chosen so that the rendered pixel board corresponds to a
  // head-on observation at some positive depth.
  const double fx = 800.0;
  const double fy = 800.0;
  const double cx = image_size.width / 2.0;
  const double cy = image_size.height / 2.0;

  out.camera_matrix = (cv::Mat_<double>(3, 3) << fx, 0.0, cx,
                                                   0.0, fy, cy,
                                                   0.0, 0.0, 1.0);
  out.dist_coeffs = cv::Mat::zeros(1, 5, CV_64F);
  return out;
}

CharucoBoardConfig defaultConfig() {
  CharucoBoardConfig cfg;
  cfg.squares_x = 7;
  cfg.squares_y = 5;
  cfg.square_length = 0.04F;
  cfg.marker_length = 0.03F;
  cfg.dictionary_name = "DICT_6X6_250";
  cfg.min_corners = 4;
  return cfg;
}

}  // namespace

// --- Dictionary parsing ------------------------------------------------------

TEST(CharucoDetectorTest, ConstructWithValidDictionary) {
  CharucoBoardConfig cfg = defaultConfig();
  EXPECT_NO_THROW({ CharucoDetector det(cfg); });
}

TEST(CharucoDetectorTest, ConstructWithAllSupportedDictionaries) {
  const char* kNames[] = {
      "DICT_4X4_50",   "DICT_4X4_100",  "DICT_4X4_250",  "DICT_4X4_1000",
      "DICT_5X5_50",   "DICT_5X5_100",  "DICT_5X5_250",  "DICT_5X5_1000",
      "DICT_6X6_50",   "DICT_6X6_100",  "DICT_6X6_250",  "DICT_6X6_1000",
      "DICT_7X7_50",   "DICT_7X7_100",  "DICT_7X7_250",  "DICT_7X7_1000",
      "DICT_ARUCO_ORIGINAL"};
  for (const char* name : kNames) {
    CharucoBoardConfig cfg = defaultConfig();
    cfg.dictionary_name = name;
    EXPECT_NO_THROW({ CharucoDetector det(cfg); })
        << "Construction failed for dictionary: " << name;
  }
}

TEST(CharucoDetectorTest, ConstructWithUnknownDictionaryThrows) {
  CharucoBoardConfig cfg = defaultConfig();
  cfg.dictionary_name = "NOT_A_REAL_DICT";
  EXPECT_THROW({ CharucoDetector det(cfg); }, std::invalid_argument);
}

// --- Detection edge cases ----------------------------------------------------

TEST(CharucoDetectorTest, EmptyImageReturnsFailure) {
  CharucoBoardConfig cfg = defaultConfig();
  CharucoDetector detector(cfg);

  cv::Mat empty_image;
  cv::Mat camera_matrix =
      (cv::Mat_<double>(3, 3) << 600, 0, 320, 0, 600, 240, 0, 0, 1);
  cv::Mat dist_coeffs = cv::Mat::zeros(1, 5, CV_64F);

  DetectionResult result = detector.detect(empty_image, camera_matrix, dist_coeffs);
  EXPECT_FALSE(result.success);
}

TEST(CharucoDetectorTest, BlankImageReturnsFailure) {
  CharucoBoardConfig cfg = defaultConfig();
  CharucoDetector detector(cfg);

  cv::Mat blank = cv::Mat::zeros(480, 640, CV_8UC3);
  cv::Mat camera_matrix =
      (cv::Mat_<double>(3, 3) << 600, 0, 320, 0, 600, 240, 0, 0, 1);
  cv::Mat dist_coeffs = cv::Mat::zeros(1, 5, CV_64F);

  DetectionResult result = detector.detect(blank, camera_matrix, dist_coeffs);
  EXPECT_FALSE(result.success) << "Blank image must not produce a detection";
}

// --- Successful detection on rendered board ----------------------------------

TEST(CharucoDetectorTest, DetectsRenderedBoardSuccessfully) {
  CharucoBoardConfig cfg = defaultConfig();
  CharucoDetector detector(cfg);

  cv::Size image_size(800, 600);
  SyntheticBoardImage synthetic =
      renderBoardHeadOn(detector, image_size, 20);

  DetectionResult result = detector.detect(synthetic.image, synthetic.camera_matrix,
                                           synthetic.dist_coeffs);

  ASSERT_TRUE(result.success) << "Detector should succeed on a clean rendered "
                                 "ChArUco board image";

  // Should find more than the minimum corner count on a full-board render.
  EXPECT_GT(result.corners_2d.size(), static_cast<size_t>(cfg.min_corners));
  EXPECT_EQ(result.corners_2d.size(), result.corners_3d.size());

  // Pose must be finite.
  for (int i = 0; i < 3; ++i) {
    EXPECT_TRUE(std::isfinite(result.rvec[i]));
    EXPECT_TRUE(std::isfinite(result.tvec[i]));
  }

  // tvec must be in front of the camera (positive Z).
  EXPECT_GT(result.tvec[2], 0.0);

  // Synthetic board was rendered noise-free → reprojection error must be small.
  EXPECT_LT(result.reprojection_error, 2.0)
      << "Reprojection error too large for a noise-free render";
}

TEST(CharucoDetectorTest, GrayscaleInputWorks) {
  CharucoBoardConfig cfg = defaultConfig();
  CharucoDetector detector(cfg);

  cv::Size image_size(800, 600);
  SyntheticBoardImage synthetic =
      renderBoardHeadOn(detector, image_size, 20);

  cv::Mat gray;
  if (synthetic.image.channels() == 3) {
    cv::cvtColor(synthetic.image, gray, cv::COLOR_BGR2GRAY);
  } else {
    gray = synthetic.image;
  }

  DetectionResult result =
      detector.detect(gray, synthetic.camera_matrix, synthetic.dist_coeffs);
  EXPECT_TRUE(result.success) << "Detector should accept single-channel input";
}

TEST(CharucoDetectorTest, CroppedBoardBelowMinCornersFails) {
  CharucoBoardConfig cfg = defaultConfig();
  cfg.min_corners = 1000;  // Unreachable threshold
  CharucoDetector detector(cfg);

  cv::Size image_size(800, 600);
  SyntheticBoardImage synthetic =
      renderBoardHeadOn(detector, image_size, 20);

  DetectionResult result = detector.detect(synthetic.image, synthetic.camera_matrix,
                                           synthetic.dist_coeffs);
  EXPECT_FALSE(result.success)
      << "Detector must respect the configured min_corners threshold";
}

TEST(CharucoDetectorTest, GetBoardReturnsConfiguredDimensions) {
  CharucoBoardConfig cfg = defaultConfig();
  CharucoDetector detector(cfg);

  auto board = detector.getBoard();
  ASSERT_TRUE(board);
  // chessboardCorners has (squares_x - 1) * (squares_y - 1) entries.
  const size_t expected =
      static_cast<size_t>(cfg.squares_x - 1) *
      static_cast<size_t>(cfg.squares_y - 1);
  EXPECT_EQ(board->chessboardCorners.size(), expected);
}
