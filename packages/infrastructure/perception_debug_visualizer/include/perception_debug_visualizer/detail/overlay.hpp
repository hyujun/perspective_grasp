// Copyright 2026 perspective_grasp
// SPDX-License-Identifier: Apache-2.0

#pragma once

#include <opencv2/core.hpp>

#include <geometry_msgs/msg/pose.hpp>
#include <perception_msgs/msg/detection_array.hpp>
#include <perception_msgs/msg/segmentation_array.hpp>

#include <array>
#include <cstdint>
#include <optional>
#include <string>
#include <vector>

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

/// 2D projection of a 6D pose's three body axes onto an image plane.
/// `origin`, `x_end`, `y_end`, `z_end` are pixel coordinates (doubles for
/// subpixel precision; callers cast to int for drawing). `track_id` labels
/// the axis triad in the overlay.
struct ProjectedAxis {
  cv::Point2d origin;
  cv::Point2d x_end;
  cv::Point2d y_end;
  cv::Point2d z_end;
  int32_t track_id{0};
};

/// Projects a 6D pose (already expressed in the camera optical frame) through
/// the pinhole intrinsics `K` (row-major 3×3) to obtain four pixel coordinates:
/// the origin plus the tips of body-frame X, Y and Z axes scaled by
/// `axis_length_m`.
///
/// Returns `std::nullopt` when the origin sits on or behind the image plane
/// (`origin.z <= 0`), when `axis_length_m <= 0`, or when `K` has a
/// non-positive focal length. Tips whose depth is non-positive are projected
/// using a clamped `z = epsilon` so the resulting segment still captures
/// direction; the drawing stage clips off-image pixels with `cv::clipLine`.
std::optional<ProjectedAxis> project_pose_axes(
    const geometry_msgs::msg::Pose& pose_in_cam,
    const std::array<double, 9>& K,
    double axis_length_m,
    int32_t track_id = 0);

/// Draws each projected axis triad onto `image`: X→red, Y→green, Z→blue, then
/// stamps `track_id` near the origin. Segments that fall entirely outside the
/// image bounds are skipped via `cv::clipLine`. No-op if `image` is empty or
/// `axes` is empty. `thickness` is clamped to `[1, 10]`.
void draw_pose_axes(cv::Mat& image,
                    const std::vector<ProjectedAxis>& axes,
                    int thickness = 2);

}  // namespace perspective_grasp::detail
