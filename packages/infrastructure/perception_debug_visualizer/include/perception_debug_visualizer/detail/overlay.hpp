// Copyright 2026 perspective_grasp
// SPDX-License-Identifier: Apache-2.0

#pragma once

#include <opencv2/core.hpp>

#include <perception_msgs/msg/detection_array.hpp>
#include <perception_msgs/msg/segmentation_array.hpp>

#include <string>

namespace perspective_grasp::detail {

/// Draws "Mode: <mode>" in the top-left corner of `image`.
/// Mutates `image` in place. No-op if `image` is empty.
void draw_mode_overlay(cv::Mat& image, const std::string& mode);

/// Draws each detection's bounding box and class label onto `image`.
/// Bounding boxes clipped to the image bounds. No-op if `image` is empty.
void draw_detections(cv::Mat& image,
                     const perception_msgs::msg::DetectionArray& detections);

/// Alpha-blends each segmentation mask onto `image` with a per-id color.
/// `alpha` is clamped to [0, 1]. Masks with a size different from `image`
/// are skipped; the function returns the number of skipped entries so the
/// caller can log. No-op (returns 0) if `image` is empty.
std::size_t draw_segmentations(
    cv::Mat& image,
    const perception_msgs::msg::SegmentationArray& segmentations,
    double alpha = 0.4);

}  // namespace perspective_grasp::detail
