// Copyright 2026 perspective_grasp
// SPDX-License-Identifier: Apache-2.0

#include "perception_debug_visualizer/detail/overlay.hpp"

#include <opencv2/imgproc.hpp>

#include <algorithm>

namespace perspective_grasp::detail {

namespace {
constexpr int kModeTextX = 10;
constexpr int kModeTextY = 30;
constexpr double kModeFontScale = 0.8;
constexpr int kModeFontThickness = 2;
}  // namespace

void draw_mode_overlay(cv::Mat& image, const std::string& mode) {
  if (image.empty()) {
    return;
  }
  cv::putText(image, "Mode: " + mode, cv::Point(kModeTextX, kModeTextY),
              cv::FONT_HERSHEY_SIMPLEX, kModeFontScale,
              cv::Scalar(0, 255, 0), kModeFontThickness);
}

void draw_detections(cv::Mat& image,
                     const perception_msgs::msg::DetectionArray& detections) {
  if (image.empty()) {
    return;
  }
  for (const auto& det : detections.detections) {
    const int x0 = std::clamp(static_cast<int>(det.bbox.x_offset), 0,
                              image.cols - 1);
    const int y0 = std::clamp(static_cast<int>(det.bbox.y_offset), 0,
                              image.rows - 1);
    const int x1 = std::clamp(
        static_cast<int>(det.bbox.x_offset + det.bbox.width), 0, image.cols - 1);
    const int y1 = std::clamp(
        static_cast<int>(det.bbox.y_offset + det.bbox.height), 0,
        image.rows - 1);
    if (x1 <= x0 || y1 <= y0) {
      continue;
    }
    cv::rectangle(image, cv::Point(x0, y0), cv::Point(x1, y1),
                  cv::Scalar(255, 0, 0), 2);
    cv::putText(image, det.class_name, cv::Point(x0, std::max(0, y0 - 5)),
                cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 0, 0), 1);
  }
}

}  // namespace perspective_grasp::detail
