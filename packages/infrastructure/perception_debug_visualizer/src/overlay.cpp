// Copyright 2026 perspective_grasp
// SPDX-License-Identifier: Apache-2.0

#include "perception_debug_visualizer/detail/overlay.hpp"

#include <opencv2/imgproc.hpp>

#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <cstdio>

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

namespace {

cv::Scalar color_from_id(int32_t id) {
  // Evenly spaced hues so adjacent ids get distinguishable colors.
  const int hue = static_cast<int>(
      static_cast<uint32_t>(id * 37) % 180u);
  cv::Mat hsv(1, 1, CV_8UC3, cv::Scalar(hue, 200, 255));
  cv::Mat bgr;
  cv::cvtColor(hsv, bgr, cv::COLOR_HSV2BGR);
  const auto px = bgr.at<cv::Vec3b>(0, 0);
  return cv::Scalar(px[0], px[1], px[2]);
}

}  // namespace

std::size_t draw_segmentations(
    cv::Mat& image,
    const perception_msgs::msg::SegmentationArray& segmentations,
    double alpha) {
  if (image.empty()) {
    return 0;
  }
  const double a = std::clamp(alpha, 0.0, 1.0);
  const double inv_a = 1.0 - a;
  std::size_t skipped = 0;

  for (const auto& seg : segmentations.segmentations) {
    const int mh = static_cast<int>(seg.mask.height);
    const int mw = static_cast<int>(seg.mask.width);
    if (mh <= 0 || mw <= 0 || seg.mask.data.empty()) {
      ++skipped;
      continue;
    }
    if (mh != image.rows || mw != image.cols) {
      ++skipped;
      continue;
    }

    // Wrap the ROS Image buffer as a mono8 cv::Mat without copying. The step
    // must be respected — cv::Mat honors `seg.mask.step`.
    cv::Mat mask(mh, mw, CV_8UC1,
                 const_cast<uint8_t*>(seg.mask.data.data()),
                 static_cast<size_t>(seg.mask.step));

    // Binarize in case the publisher emits non-{0,255} values.
    cv::Mat mask_bin;
    cv::threshold(mask, mask_bin, 0, 255, cv::THRESH_BINARY);

    const cv::Scalar color = color_from_id(seg.id);

    // Tint only the masked region: out = (1-a)*image + a*color, where mask>0.
    cv::Mat tinted(image.size(), image.type(), color);
    cv::Mat blended;
    cv::addWeighted(image, inv_a, tinted, a, 0.0, blended);
    blended.copyTo(image, mask_bin);

    // Draw bbox outline + id/confidence label in the same color.
    const int x0 = std::clamp(static_cast<int>(seg.bbox.x_offset), 0,
                              image.cols - 1);
    const int y0 = std::clamp(static_cast<int>(seg.bbox.y_offset), 0,
                              image.rows - 1);
    const int x1 = std::clamp(
        static_cast<int>(seg.bbox.x_offset + seg.bbox.width), 0,
        image.cols - 1);
    const int y1 = std::clamp(
        static_cast<int>(seg.bbox.y_offset + seg.bbox.height), 0,
        image.rows - 1);
    if (x1 > x0 && y1 > y0) {
      cv::rectangle(image, cv::Point(x0, y0), cv::Point(x1, y1), color, 1);
      char label[64];
      std::snprintf(label, sizeof(label), "id=%d %.2f", seg.id, seg.confidence);
      cv::putText(image, label, cv::Point(x0, std::max(0, y0 - 5)),
                  cv::FONT_HERSHEY_SIMPLEX, 0.4, color, 1);
    }
  }
  return skipped;
}

}  // namespace perspective_grasp::detail
