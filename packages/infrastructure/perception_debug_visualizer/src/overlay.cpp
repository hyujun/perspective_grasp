// Copyright 2026 perspective_grasp
// SPDX-License-Identifier: Apache-2.0

#include "perception_debug_visualizer/detail/overlay.hpp"

#include <opencv2/imgproc.hpp>

#include <algorithm>
#include <array>
#include <cstdint>

namespace perspective_grasp::detail {

namespace {
constexpr int kModeTextX = 10;
constexpr int kModeTextY = 30;
constexpr double kModeFontScale = 0.8;
constexpr int kModeFontThickness = 2;

// BGR palette for instance masks. Deliberately avoids pure blue (detection
// bbox) and pure green (mode text) so overlays remain distinguishable.
// cv::Vec3b isn't a literal type in this OpenCV build, so the palette is
// stored as raw BGR bytes; we wrap at the call site.
constexpr std::array<std::array<uint8_t, 3>, 8> kMaskPalette = {{
    {{  0,   0, 255}},  // red
    {{  0, 255, 255}},  // yellow
    {{255,   0, 255}},  // magenta
    {{  0, 128, 255}},  // orange
    {{128,   0, 255}},  // pink
    {{128, 255,   0}},  // lime
    {{255, 128, 128}},  // light blue
    {{128, 128, 255}},  // salmon
}};
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

void draw_masks(cv::Mat& image,
                const perception_msgs::msg::SegmentationArray& masks,
                float alpha) {
  if (image.empty() || alpha <= 0.0f) {
    return;
  }
  const float a = std::min(alpha, 1.0f);
  const float inv_a = 1.0f - a;

  for (const auto& seg : masks.segmentations) {
    const auto& mask_msg = seg.mask;
    if (mask_msg.encoding != "mono8") {
      continue;
    }
    if (static_cast<int>(mask_msg.width) != image.cols ||
        static_cast<int>(mask_msg.height) != image.rows) {
      continue;
    }
    const std::size_t expected = static_cast<std::size_t>(mask_msg.step) *
                                 static_cast<std::size_t>(mask_msg.height);
    if (mask_msg.data.size() < expected || mask_msg.step < mask_msg.width) {
      continue;
    }

    // Wrap the ROS mask buffer without copying. const_cast is safe — we only
    // read from mask_mat below.
    cv::Mat mask_mat(static_cast<int>(mask_msg.height),
                     static_cast<int>(mask_msg.width), CV_8UC1,
                     const_cast<uint8_t*>(mask_msg.data.data()),
                     static_cast<std::size_t>(mask_msg.step));

    const std::size_t palette_index =
        static_cast<std::size_t>(
            seg.id < 0 ? -(seg.id + 1) : seg.id) % kMaskPalette.size();
    const auto& color = kMaskPalette[palette_index];

    for (int y = 0; y < image.rows; ++y) {
      const uint8_t* mask_row = mask_mat.ptr<uint8_t>(y);
      cv::Vec3b* img_row = image.ptr<cv::Vec3b>(y);
      for (int x = 0; x < image.cols; ++x) {
        if (mask_row[x] == 0) {
          continue;
        }
        for (int c = 0; c < 3; ++c) {
          img_row[x][c] = cv::saturate_cast<uint8_t>(
              inv_a * static_cast<float>(img_row[x][c]) +
              a * static_cast<float>(color[c]));
        }
      }
    }
  }
}

}  // namespace perspective_grasp::detail
