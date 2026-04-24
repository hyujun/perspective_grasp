// Copyright 2026 perspective_grasp
// SPDX-License-Identifier: Apache-2.0

#include "perception_debug_visualizer/detail/overlay.hpp"

#include <opencv2/imgproc.hpp>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <string>

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

namespace {

constexpr double kPoseAxisDepthEpsilon = 1e-4;
constexpr int kPoseAxisMinThickness = 1;
constexpr int kPoseAxisMaxThickness = 10;

struct Vec3 { double x, y, z; };

// Rotate a body-frame unit axis by a quaternion (x, y, z, w).
// Derived from R = I + 2w[ω]× + 2[ω]×² applied to the canonical basis.
Vec3 rotate_x_axis(double qx, double qy, double qz, double qw) {
  return {1.0 - 2.0 * (qy * qy + qz * qz),
          2.0 * (qx * qy + qw * qz),
          2.0 * (qx * qz - qw * qy)};
}
Vec3 rotate_y_axis(double qx, double qy, double qz, double qw) {
  return {2.0 * (qx * qy - qw * qz),
          1.0 - 2.0 * (qx * qx + qz * qz),
          2.0 * (qy * qz + qw * qx)};
}
Vec3 rotate_z_axis(double qx, double qy, double qz, double qw) {
  return {2.0 * (qx * qz + qw * qy),
          2.0 * (qy * qz - qw * qx),
          1.0 - 2.0 * (qx * qx + qy * qy)};
}

cv::Point2d project(const Vec3& p, double fx, double fy, double cx, double cy) {
  const double z = std::max(p.z, kPoseAxisDepthEpsilon);
  return {fx * p.x / z + cx, fy * p.y / z + cy};
}

}  // namespace

std::optional<ProjectedAxis> project_pose_axes(
    const geometry_msgs::msg::Pose& pose_in_cam,
    const std::array<double, 9>& K,
    double axis_length_m,
    int32_t track_id) {
  if (axis_length_m <= 0.0) {
    return std::nullopt;
  }
  const double fx = K[0];
  const double fy = K[4];
  const double cx = K[2];
  const double cy = K[5];
  if (fx <= 0.0 || fy <= 0.0) {
    return std::nullopt;
  }

  const Vec3 origin{pose_in_cam.position.x, pose_in_cam.position.y,
                    pose_in_cam.position.z};
  if (origin.z <= kPoseAxisDepthEpsilon) {
    return std::nullopt;
  }

  const double qx = pose_in_cam.orientation.x;
  const double qy = pose_in_cam.orientation.y;
  const double qz = pose_in_cam.orientation.z;
  const double qw = pose_in_cam.orientation.w;

  const Vec3 ux = rotate_x_axis(qx, qy, qz, qw);
  const Vec3 uy = rotate_y_axis(qx, qy, qz, qw);
  const Vec3 uz = rotate_z_axis(qx, qy, qz, qw);

  const Vec3 tip_x{origin.x + axis_length_m * ux.x,
                   origin.y + axis_length_m * ux.y,
                   origin.z + axis_length_m * ux.z};
  const Vec3 tip_y{origin.x + axis_length_m * uy.x,
                   origin.y + axis_length_m * uy.y,
                   origin.z + axis_length_m * uy.z};
  const Vec3 tip_z{origin.x + axis_length_m * uz.x,
                   origin.y + axis_length_m * uz.y,
                   origin.z + axis_length_m * uz.z};

  ProjectedAxis out;
  out.origin = project(origin, fx, fy, cx, cy);
  out.x_end = project(tip_x, fx, fy, cx, cy);
  out.y_end = project(tip_y, fx, fy, cx, cy);
  out.z_end = project(tip_z, fx, fy, cx, cy);
  out.track_id = track_id;
  return out;
}

void draw_pose_axes(cv::Mat& image,
                    const std::vector<ProjectedAxis>& axes,
                    int thickness) {
  if (image.empty() || axes.empty()) {
    return;
  }
  const int t = std::clamp(thickness, kPoseAxisMinThickness,
                           kPoseAxisMaxThickness);
  const cv::Rect bounds(0, 0, image.cols, image.rows);

  auto to_point = [](const cv::Point2d& p) {
    return cv::Point(static_cast<int>(std::lround(p.x)),
                     static_cast<int>(std::lround(p.y)));
  };

  for (const auto& ax : axes) {
    // BGR. X→red, Y→green, Z→blue. Matches RViz axis convention.
    const std::array<std::pair<cv::Point2d, cv::Scalar>, 3> segs = {{
        {ax.x_end, cv::Scalar(0, 0, 255)},
        {ax.y_end, cv::Scalar(0, 255, 0)},
        {ax.z_end, cv::Scalar(255, 0, 0)},
    }};
    for (const auto& [tip, color] : segs) {
      cv::Point p1 = to_point(ax.origin);
      cv::Point p2 = to_point(tip);
      // cv::clipLine rewrites p1/p2 in place; returns false if segment is
      // entirely outside the image bounds — skip in that case.
      if (!cv::clipLine(bounds, p1, p2)) {
        continue;
      }
      cv::line(image, p1, p2, color, t, cv::LINE_AA);
    }

    // Label with track id near origin (skip if origin far outside image).
    const cv::Point origin_px = to_point(ax.origin);
    if (bounds.contains(origin_px)) {
      const std::string label = "#" + std::to_string(ax.track_id);
      cv::putText(image, label,
                  origin_px + cv::Point(6, -6),
                  cv::FONT_HERSHEY_SIMPLEX, 0.4,
                  cv::Scalar(255, 255, 255), 1, cv::LINE_AA);
    }
  }
}

}  // namespace perspective_grasp::detail
