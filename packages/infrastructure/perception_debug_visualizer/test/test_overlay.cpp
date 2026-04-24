// Copyright 2026 perspective_grasp
// SPDX-License-Identifier: Apache-2.0

#include "perception_debug_visualizer/detail/overlay.hpp"

#include <gtest/gtest.h>

#include <opencv2/core.hpp>

#include <geometry_msgs/msg/pose.hpp>
#include <perception_msgs/msg/detection.hpp>
#include <perception_msgs/msg/detection_array.hpp>
#include <perception_msgs/msg/segmentation.hpp>
#include <perception_msgs/msg/segmentation_array.hpp>

#include <array>
#include <cmath>
#include <vector>

namespace {

using perspective_grasp::detail::draw_detections;
using perspective_grasp::detail::draw_masks;
using perspective_grasp::detail::draw_mode_overlay;
using perspective_grasp::detail::draw_pose_axes;
using perspective_grasp::detail::project_pose_axes;
using perspective_grasp::detail::ProjectedAxis;

std::array<double, 9> makeIntrinsics(double fx, double fy, double cx,
                                     double cy) {
  return {fx, 0.0, cx, 0.0, fy, cy, 0.0, 0.0, 1.0};
}

geometry_msgs::msg::Pose makePose(double x, double y, double z,
                                  double qx = 0.0, double qy = 0.0,
                                  double qz = 0.0, double qw = 1.0) {
  geometry_msgs::msg::Pose p;
  p.position.x = x;
  p.position.y = y;
  p.position.z = z;
  p.orientation.x = qx;
  p.orientation.y = qy;
  p.orientation.z = qz;
  p.orientation.w = qw;
  return p;
}

cv::Mat makeBlankImage(int rows = 480, int cols = 640) {
  return cv::Mat::zeros(rows, cols, CV_8UC3);
}

perception_msgs::msg::Detection makeDetection(uint32_t x, uint32_t y,
                                              uint32_t w, uint32_t h,
                                              const std::string& cls) {
  perception_msgs::msg::Detection det;
  det.bbox.x_offset = x;
  det.bbox.y_offset = y;
  det.bbox.width = w;
  det.bbox.height = h;
  det.class_name = cls;
  return det;
}

// Fills pixels inside [y0,y1)×[x0,x1) with 255 in a mono8 Image sized rows×cols.
perception_msgs::msg::Segmentation makeMaskSegmentation(
    int rows, int cols, int x0, int y0, int x1, int y1, int32_t id,
    const std::string& encoding = "mono8") {
  perception_msgs::msg::Segmentation seg;
  seg.id = id;
  seg.mask.width = static_cast<uint32_t>(cols);
  seg.mask.height = static_cast<uint32_t>(rows);
  seg.mask.encoding = encoding;
  seg.mask.step = static_cast<uint32_t>(cols);
  seg.mask.data.assign(static_cast<size_t>(rows * cols), 0);
  for (int y = std::max(0, y0); y < std::min(rows, y1); ++y) {
    for (int x = std::max(0, x0); x < std::min(cols, x1); ++x) {
      seg.mask.data[static_cast<size_t>(y * cols + x)] = 255;
    }
  }
  return seg;
}

}  // namespace

// --- draw_mode_overlay -------------------------------------------------------

TEST(OverlayTest, DrawModeOverlayEmptyImageNoOp) {
  cv::Mat empty;
  EXPECT_NO_THROW(draw_mode_overlay(empty, "NORMAL"));
  EXPECT_TRUE(empty.empty());
}

TEST(OverlayTest, DrawModeOverlayModifiesPixels) {
  cv::Mat img = makeBlankImage();
  const cv::Mat before = img.clone();
  draw_mode_overlay(img, "NORMAL");

  // At least one pixel must differ from the blank original.
  cv::Mat diff;
  cv::absdiff(img, before, diff);
  EXPECT_GT(cv::countNonZero(diff.reshape(1)), 0)
      << "draw_mode_overlay should paint text onto the image";
}

TEST(OverlayTest, DrawModeOverlayPaintsInTopBand) {
  cv::Mat img = makeBlankImage();
  draw_mode_overlay(img, "HIGH_PRECISION");

  // Top band (y < 60) should contain painted pixels; bottom band (y > 200)
  // should remain untouched.
  cv::Mat top = img.rowRange(0, 60);
  cv::Mat bottom = img.rowRange(200, img.rows);
  EXPECT_GT(cv::countNonZero(top.reshape(1)), 0);
  EXPECT_EQ(cv::countNonZero(bottom.reshape(1)), 0);
}

TEST(OverlayTest, DrawModeOverlayGreenTextColor) {
  cv::Mat img = makeBlankImage();
  draw_mode_overlay(img, "X");

  // Scan the top region: any non-zero pixel should be (0, G, 0) in BGR.
  for (int y = 0; y < 60; ++y) {
    for (int x = 0; x < img.cols; ++x) {
      auto px = img.at<cv::Vec3b>(y, x);
      if (px[0] == 0 && px[1] == 0 && px[2] == 0) continue;
      EXPECT_EQ(px[0], 0) << "Blue channel should stay 0 for green text";
      EXPECT_EQ(px[2], 0) << "Red channel should stay 0 for green text";
      EXPECT_GT(px[1], 0) << "Green channel should be set";
    }
  }
}

TEST(OverlayTest, DrawModeOverlayDoesNotResizeImage) {
  cv::Mat img = makeBlankImage(300, 400);
  draw_mode_overlay(img, "NORMAL");
  EXPECT_EQ(img.rows, 300);
  EXPECT_EQ(img.cols, 400);
  EXPECT_EQ(img.channels(), 3);
}

// --- draw_detections ---------------------------------------------------------

TEST(OverlayTest, DrawDetectionsEmptyArrayLeavesImageUnchanged) {
  cv::Mat img = makeBlankImage();
  const cv::Mat before = img.clone();
  perception_msgs::msg::DetectionArray dets;
  draw_detections(img, dets);

  cv::Mat diff;
  cv::absdiff(img, before, diff);
  EXPECT_EQ(cv::countNonZero(diff.reshape(1)), 0);
}

TEST(OverlayTest, DrawDetectionsSingleBox) {
  cv::Mat img = makeBlankImage();
  perception_msgs::msg::DetectionArray dets;
  dets.detections.push_back(makeDetection(100, 100, 200, 150, "bottle"));
  draw_detections(img, dets);

  // Pixels must have been drawn somewhere inside the bounding region.
  cv::Mat roi = img(cv::Rect(100, 100, 200, 150));
  EXPECT_GT(cv::countNonZero(roi.reshape(1)), 0);
}

TEST(OverlayTest, DrawDetectionsClampsToImageBounds) {
  cv::Mat img = makeBlankImage();
  perception_msgs::msg::DetectionArray dets;
  // Bbox extends past image bounds — must not crash or paint outside.
  dets.detections.push_back(
      makeDetection(600, 400, 200, 200, "out_of_bounds"));
  EXPECT_NO_THROW(draw_detections(img, dets));
}

TEST(OverlayTest, DrawDetectionsZeroSizedBoxSkipped) {
  cv::Mat img = makeBlankImage();
  const cv::Mat before = img.clone();
  perception_msgs::msg::DetectionArray dets;
  // Width and height zero — after clamping, x1==x0 so nothing to draw.
  dets.detections.push_back(makeDetection(50, 50, 0, 0, "empty"));
  draw_detections(img, dets);

  cv::Mat diff;
  cv::absdiff(img, before, diff);
  EXPECT_EQ(cv::countNonZero(diff.reshape(1)), 0)
      << "Zero-sized bbox must not paint pixels";
}

TEST(OverlayTest, DrawDetectionsEmptyImageNoOp) {
  cv::Mat empty;
  perception_msgs::msg::DetectionArray dets;
  dets.detections.push_back(makeDetection(0, 0, 10, 10, "bottle"));
  EXPECT_NO_THROW(draw_detections(empty, dets));
  EXPECT_TRUE(empty.empty());
}

TEST(OverlayTest, DrawDetectionsMultipleBoxes) {
  cv::Mat img = makeBlankImage();
  perception_msgs::msg::DetectionArray dets;
  dets.detections.push_back(makeDetection(10, 10, 50, 50, "a"));
  dets.detections.push_back(makeDetection(200, 200, 100, 80, "b"));
  dets.detections.push_back(makeDetection(400, 300, 60, 60, "c"));
  draw_detections(img, dets);

  for (const auto& det : dets.detections) {
    cv::Rect r(static_cast<int>(det.bbox.x_offset),
               static_cast<int>(det.bbox.y_offset),
               static_cast<int>(det.bbox.width),
               static_cast<int>(det.bbox.height));
    r &= cv::Rect(0, 0, img.cols, img.rows);
    cv::Mat roi = img(r);
    EXPECT_GT(cv::countNonZero(roi.reshape(1)), 0)
        << "Expected painted pixels inside bbox for class " << det.class_name;
  }
}

// --- draw_masks --------------------------------------------------------------

TEST(OverlayTest, DrawMasksEmptyArrayLeavesImageUnchanged) {
  cv::Mat img = makeBlankImage();
  const cv::Mat before = img.clone();
  perception_msgs::msg::SegmentationArray masks;
  draw_masks(img, masks);

  cv::Mat diff;
  cv::absdiff(img, before, diff);
  EXPECT_EQ(cv::countNonZero(diff.reshape(1)), 0);
}

TEST(OverlayTest, DrawMasksEmptyImageNoOp) {
  cv::Mat empty;
  perception_msgs::msg::SegmentationArray masks;
  masks.segmentations.push_back(
      makeMaskSegmentation(480, 640, 0, 0, 10, 10, /*id=*/0));
  EXPECT_NO_THROW(draw_masks(empty, masks));
  EXPECT_TRUE(empty.empty());
}

TEST(OverlayTest, DrawMasksRespectsMaskBoundary) {
  cv::Mat img = makeBlankImage(240, 320);
  perception_msgs::msg::SegmentationArray masks;
  // Paint inside [50,150)×[60,140)
  masks.segmentations.push_back(
      makeMaskSegmentation(240, 320, 60, 50, 140, 150, /*id=*/0));
  draw_masks(img, masks, 1.0f);

  // Inside mask: non-zero pixels.
  cv::Rect inside(60, 50, 80, 100);
  EXPECT_GT(cv::countNonZero(img(inside).reshape(1)), 0);

  // Outside mask (top band row 0-49): untouched.
  cv::Mat above = img.rowRange(0, 50);
  EXPECT_EQ(cv::countNonZero(above.reshape(1)), 0);

  // Outside mask (bottom band row 150-239): untouched.
  cv::Mat below = img.rowRange(150, img.rows);
  EXPECT_EQ(cv::countNonZero(below.reshape(1)), 0);
}

TEST(OverlayTest, DrawMasksDimensionMismatchIsSkipped) {
  cv::Mat img = makeBlankImage(240, 320);
  const cv::Mat before = img.clone();
  perception_msgs::msg::SegmentationArray masks;
  // Mask size 100×100 doesn't match 240×320 image — skip silently.
  masks.segmentations.push_back(
      makeMaskSegmentation(100, 100, 0, 0, 100, 100, /*id=*/0));
  EXPECT_NO_THROW(draw_masks(img, masks));

  cv::Mat diff;
  cv::absdiff(img, before, diff);
  EXPECT_EQ(cv::countNonZero(diff.reshape(1)), 0);
}

TEST(OverlayTest, DrawMasksUnsupportedEncodingIsSkipped) {
  cv::Mat img = makeBlankImage(240, 320);
  const cv::Mat before = img.clone();
  perception_msgs::msg::SegmentationArray masks;
  auto seg = makeMaskSegmentation(240, 320, 10, 10, 50, 50, /*id=*/0);
  seg.mask.encoding = "bgr8";  // unsupported
  masks.segmentations.push_back(seg);
  draw_masks(img, masks);

  cv::Mat diff;
  cv::absdiff(img, before, diff);
  EXPECT_EQ(cv::countNonZero(diff.reshape(1)), 0);
}

TEST(OverlayTest, DrawMasksAlphaZeroNoChange) {
  cv::Mat img = makeBlankImage(240, 320);
  const cv::Mat before = img.clone();
  perception_msgs::msg::SegmentationArray masks;
  masks.segmentations.push_back(
      makeMaskSegmentation(240, 320, 10, 10, 100, 100, /*id=*/0));
  draw_masks(img, masks, 0.0f);

  cv::Mat diff;
  cv::absdiff(img, before, diff);
  EXPECT_EQ(cv::countNonZero(diff.reshape(1)), 0);
}

TEST(OverlayTest, DrawMasksAlphaOneFullyReplacesColor) {
  cv::Mat img = makeBlankImage(240, 320);
  perception_msgs::msg::SegmentationArray masks;
  // id=0 → palette[0] which is red (BGR: 0,0,255).
  masks.segmentations.push_back(
      makeMaskSegmentation(240, 320, 10, 10, 50, 50, /*id=*/0));
  draw_masks(img, masks, 1.0f);

  auto px = img.at<cv::Vec3b>(30, 30);
  EXPECT_EQ(px[0], 0);
  EXPECT_EQ(px[1], 0);
  EXPECT_EQ(px[2], 255);
}

TEST(OverlayTest, DrawMasksDifferentIdsUseDifferentColors) {
  cv::Mat img = makeBlankImage(240, 320);
  perception_msgs::msg::SegmentationArray masks;
  // Two non-overlapping masks at different regions, different ids.
  masks.segmentations.push_back(
      makeMaskSegmentation(240, 320, 10, 10, 40, 40, /*id=*/0));
  masks.segmentations.push_back(
      makeMaskSegmentation(240, 320, 100, 100, 140, 140, /*id=*/1));
  draw_masks(img, masks, 1.0f);

  auto a = img.at<cv::Vec3b>(25, 25);    // inside id=0 region
  auto b = img.at<cv::Vec3b>(120, 120);  // inside id=1 region
  EXPECT_NE(a, b)
      << "Masks with different ids should map to different palette colors";
}

TEST(OverlayTest, DrawMasksNegativeIdDoesNotCrash) {
  cv::Mat img = makeBlankImage(240, 320);
  perception_msgs::msg::SegmentationArray masks;
  masks.segmentations.push_back(
      makeMaskSegmentation(240, 320, 0, 0, 30, 30, /*id=*/-7));
  EXPECT_NO_THROW(draw_masks(img, masks, 1.0f));
  EXPECT_GT(cv::countNonZero(img.reshape(1)), 0);
}

// --- project_pose_axes -------------------------------------------------------

TEST(OverlayTest, ProjectPoseAxesIdentityAtOpticalAxisHitsPrincipalPoint) {
  const auto K = makeIntrinsics(500.0, 500.0, 320.0, 240.0);
  // Pose at (0, 0, 1) with identity orientation: origin projects to (cx, cy).
  auto proj = project_pose_axes(makePose(0, 0, 1.0), K, /*axis_length=*/0.1);
  ASSERT_TRUE(proj.has_value());
  EXPECT_NEAR(proj->origin.x, 320.0, 1e-6);
  EXPECT_NEAR(proj->origin.y, 240.0, 1e-6);
  // X body axis tip at (0.1, 0, 1) → u = 500*0.1/1 + 320 = 370
  EXPECT_NEAR(proj->x_end.x, 370.0, 1e-6);
  EXPECT_NEAR(proj->x_end.y, 240.0, 1e-6);
  // Y body axis tip at (0, 0.1, 1) → v = 500*0.1/1 + 240 = 290
  EXPECT_NEAR(proj->y_end.x, 320.0, 1e-6);
  EXPECT_NEAR(proj->y_end.y, 290.0, 1e-6);
  // Z body axis tip at (0, 0, 1.1) collapses onto the principal point.
  EXPECT_NEAR(proj->z_end.x, 320.0, 1e-6);
  EXPECT_NEAR(proj->z_end.y, 240.0, 1e-6);
}

TEST(OverlayTest, ProjectPoseAxesTrackIdIsCarried) {
  const auto K = makeIntrinsics(500.0, 500.0, 320.0, 240.0);
  auto proj = project_pose_axes(makePose(0, 0, 1.0), K, 0.05, /*id=*/42);
  ASSERT_TRUE(proj.has_value());
  EXPECT_EQ(proj->track_id, 42);
}

TEST(OverlayTest, ProjectPoseAxesRejectsOriginBehindCamera) {
  const auto K = makeIntrinsics(500.0, 500.0, 320.0, 240.0);
  EXPECT_FALSE(project_pose_axes(makePose(0, 0, -0.1), K, 0.05).has_value());
  EXPECT_FALSE(project_pose_axes(makePose(0, 0, 0.0), K, 0.05).has_value());
}

TEST(OverlayTest, ProjectPoseAxesRejectsInvalidInputs) {
  const auto K = makeIntrinsics(500.0, 500.0, 320.0, 240.0);
  // Non-positive axis length.
  EXPECT_FALSE(project_pose_axes(makePose(0, 0, 1.0), K, 0.0).has_value());
  EXPECT_FALSE(project_pose_axes(makePose(0, 0, 1.0), K, -0.05).has_value());
  // Zero focal length.
  const auto bad_K = makeIntrinsics(0.0, 500.0, 320.0, 240.0);
  EXPECT_FALSE(project_pose_axes(makePose(0, 0, 1.0), bad_K, 0.05).has_value());
}

TEST(OverlayTest, ProjectPoseAxes90DegYawSwapsXEnd) {
  const auto K = makeIntrinsics(500.0, 500.0, 320.0, 240.0);
  // 90° rotation about body Z: body X maps to world +Y. Quaternion
  // (0, 0, sin45, cos45).
  const double s = std::sin(M_PI / 4.0);
  const double c = std::cos(M_PI / 4.0);
  auto proj = project_pose_axes(makePose(0, 0, 1.0, 0, 0, s, c), K, 0.1);
  ASSERT_TRUE(proj.has_value());
  // Body-X tip now lives at world (0, 0.1, 1) → v = 290 (not u = 370).
  EXPECT_NEAR(proj->x_end.x, 320.0, 1e-4);
  EXPECT_NEAR(proj->x_end.y, 290.0, 1e-4);
}

// --- draw_pose_axes ----------------------------------------------------------

ProjectedAxis makeCenteredAxis(int32_t id = 0) {
  ProjectedAxis a;
  a.origin = {320.0, 240.0};
  a.x_end = {370.0, 240.0};
  a.y_end = {320.0, 290.0};
  a.z_end = {300.0, 220.0};
  a.track_id = id;
  return a;
}

TEST(OverlayTest, DrawPoseAxesEmptyImageNoOp) {
  cv::Mat empty;
  std::vector<ProjectedAxis> axes{makeCenteredAxis()};
  EXPECT_NO_THROW(draw_pose_axes(empty, axes));
  EXPECT_TRUE(empty.empty());
}

TEST(OverlayTest, DrawPoseAxesEmptyVectorLeavesImageUnchanged) {
  cv::Mat img = makeBlankImage();
  const cv::Mat before = img.clone();
  draw_pose_axes(img, {});
  cv::Mat diff;
  cv::absdiff(img, before, diff);
  EXPECT_EQ(cv::countNonZero(diff.reshape(1)), 0);
}

TEST(OverlayTest, DrawPoseAxesPaintsThreeChannelColors) {
  cv::Mat img = makeBlankImage();
  // Widely-separated endpoints so the three colored lines don't overlap.
  ProjectedAxis a;
  a.origin = {320.0, 240.0};
  a.x_end = {420.0, 240.0};  // →right, red
  a.y_end = {320.0, 340.0};  // ↓down, green
  a.z_end = {220.0, 140.0};  // ↖diagonal, blue
  draw_pose_axes(img, {a}, 3);

  bool saw_red = false, saw_green = false, saw_blue = false;
  for (int y = 0; y < img.rows; ++y) {
    for (int x = 0; x < img.cols; ++x) {
      const auto px = img.at<cv::Vec3b>(y, x);
      if (px[2] > px[0] && px[2] > px[1]) saw_red = true;
      if (px[1] > px[0] && px[1] > px[2]) saw_green = true;
      if (px[0] > px[1] && px[0] > px[2]) saw_blue = true;
    }
  }
  EXPECT_TRUE(saw_red) << "Expected red (X) pixels";
  EXPECT_TRUE(saw_green) << "Expected green (Y) pixels";
  EXPECT_TRUE(saw_blue) << "Expected blue (Z) pixels";
}

TEST(OverlayTest, DrawPoseAxesOutOfBoundsSkipsWithoutCrash) {
  cv::Mat img = makeBlankImage(240, 320);
  ProjectedAxis a;
  // Origin and all tips far outside the image — clipLine should reject.
  a.origin = {-1000.0, -1000.0};
  a.x_end = {-800.0, -1000.0};
  a.y_end = {-1000.0, -800.0};
  a.z_end = {-900.0, -900.0};
  const cv::Mat before = img.clone();
  EXPECT_NO_THROW(draw_pose_axes(img, {a}));
  cv::Mat diff;
  cv::absdiff(img, before, diff);
  EXPECT_EQ(cv::countNonZero(diff.reshape(1)), 0);
}

TEST(OverlayTest, DrawPoseAxesMultipleTriadsAllRender) {
  cv::Mat img = makeBlankImage();
  std::vector<ProjectedAxis> axes;
  for (int i = 0; i < 3; ++i) {
    ProjectedAxis a;
    const double cx = 100.0 + 150.0 * i;
    a.origin = {cx, 200.0};
    a.x_end = {cx + 30.0, 200.0};
    a.y_end = {cx, 230.0};
    a.z_end = {cx + 10.0, 190.0};
    a.track_id = i;
    axes.push_back(a);
  }
  draw_pose_axes(img, axes);

  // Expect painted pixels in each ROI around each axis origin.
  for (const auto& a : axes) {
    cv::Rect roi(static_cast<int>(a.origin.x) - 40,
                 static_cast<int>(a.origin.y) - 40, 80, 80);
    roi &= cv::Rect(0, 0, img.cols, img.rows);
    EXPECT_GT(cv::countNonZero(img(roi).reshape(1)), 0);
  }
}
