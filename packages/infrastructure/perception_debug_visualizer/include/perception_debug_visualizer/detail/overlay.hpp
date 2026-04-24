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

/// Alpha-blends each mono8 instance mask in `masks` onto `image` using a
/// palette keyed by `Segmentation::id`. Masks whose dimensions differ from
/// `image`, or whose encoding is not ``mono8``, are silently skipped. No-op
/// if `image` is empty or `alpha` is non-positive. `alpha` is clamped to
/// [0, 1].
void draw_masks(cv::Mat& image,
                const perception_msgs::msg::SegmentationArray& masks,
                float alpha = 0.4f);

}  // namespace perspective_grasp::detail
