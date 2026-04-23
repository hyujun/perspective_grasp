// Copyright 2026 perspective_grasp
// SPDX-License-Identifier: Apache-2.0

#include "perception_debug_visualizer/detail/overlay.hpp"

#include <gtest/gtest.h>

#include <opencv2/core.hpp>

#include <perception_msgs/msg/detection.hpp>
#include <perception_msgs/msg/detection_array.hpp>
#include <perception_msgs/msg/segmentation.hpp>
#include <perception_msgs/msg/segmentation_array.hpp>

#include <cstddef>
#include <cstdint>
#include <vector>

namespace {

using perspective_grasp::detail::draw_detections;
using perspective_grasp::detail::draw_mode_overlay;
using perspective_grasp::detail::draw_segmentations;

perception_msgs::msg::Segmentation makeSegmentation(
    int32_t id, int rows, int cols,
    const cv::Rect& fg_rect, uint32_t bbox_x = 0,
    uint32_t bbox_y = 0, uint32_t bbox_w = 0, uint32_t bbox_h = 0,
    float confidence = 0.9f) {
  perception_msgs::msg::Segmentation seg;
  seg.id = id;
  seg.confidence = confidence;
  seg.bbox.x_offset = bbox_x;
  seg.bbox.y_offset = bbox_y;
  seg.bbox.width = bbox_w;
  seg.bbox.height = bbox_h;

  seg.mask.height = static_cast<uint32_t>(rows);
  seg.mask.width = static_cast<uint32_t>(cols);
  seg.mask.encoding = "mono8";
  seg.mask.is_bigendian = 0;
  seg.mask.step = static_cast<uint32_t>(cols);
  seg.mask.data.assign(static_cast<std::size_t>(rows * cols), 0);

  cv::Rect clipped = fg_rect & cv::Rect(0, 0, cols, rows);
  for (int y = clipped.y; y < clipped.y + clipped.height; ++y) {
    for (int x = clipped.x; x < clipped.x + clipped.width; ++x) {
      seg.mask.data[static_cast<std::size_t>(y * cols + x)] = 255;
    }
  }
  return seg;
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

// --- draw_segmentations ------------------------------------------------------

TEST(OverlayTest, DrawSegmentationsEmptyImageNoOp) {
  cv::Mat empty;
  perception_msgs::msg::SegmentationArray arr;
  arr.segmentations.push_back(makeSegmentation(1, 10, 10, cv::Rect(0, 0, 5, 5)));
  EXPECT_EQ(draw_segmentations(empty, arr), 0u);
  EXPECT_TRUE(empty.empty());
}

TEST(OverlayTest, DrawSegmentationsEmptyArrayLeavesImageUnchanged) {
  cv::Mat img = makeBlankImage();
  const cv::Mat before = img.clone();
  perception_msgs::msg::SegmentationArray arr;
  EXPECT_EQ(draw_segmentations(img, arr), 0u);

  cv::Mat diff;
  cv::absdiff(img, before, diff);
  EXPECT_EQ(cv::countNonZero(diff.reshape(1)), 0);
}

TEST(OverlayTest, DrawSegmentationsSizeMismatchCounted) {
  cv::Mat img = makeBlankImage(240, 320);
  const cv::Mat before = img.clone();
  perception_msgs::msg::SegmentationArray arr;
  // Mask is 480x640 but image is 240x320 — must be skipped, not resized.
  arr.segmentations.push_back(
      makeSegmentation(1, 480, 640, cv::Rect(0, 0, 10, 10)));
  EXPECT_EQ(draw_segmentations(img, arr), 1u);

  cv::Mat diff;
  cv::absdiff(img, before, diff);
  EXPECT_EQ(cv::countNonZero(diff.reshape(1)), 0)
      << "Size-mismatched mask must not paint any pixel";
}

TEST(OverlayTest, DrawSegmentationsEmptyMaskDataCounted) {
  cv::Mat img = makeBlankImage();
  perception_msgs::msg::SegmentationArray arr;
  perception_msgs::msg::Segmentation seg;
  seg.id = 7;
  seg.mask.height = 0;
  seg.mask.width = 0;
  arr.segmentations.push_back(seg);
  EXPECT_EQ(draw_segmentations(img, arr), 1u);
}

TEST(OverlayTest, DrawSegmentationsBlendsForegroundPixels) {
  cv::Mat img = makeBlankImage();
  perception_msgs::msg::SegmentationArray arr;
  arr.segmentations.push_back(
      makeSegmentation(3, img.rows, img.cols,
                       cv::Rect(100, 100, 50, 50),
                       100, 100, 50, 50));
  EXPECT_EQ(draw_segmentations(img, arr, 0.5), 0u);

  // Interior foreground pixel must be tinted away from black.
  const auto inside = img.at<cv::Vec3b>(125, 125);
  EXPECT_GT(inside[0] + inside[1] + inside[2], 0)
      << "Mask foreground should be blended with color";

  // Background pixel far from the mask stays black (rectangle outline drawn
  // on bbox edge may paint elsewhere, so sample from a safe corner).
  const auto outside = img.at<cv::Vec3b>(10, 10);
  EXPECT_EQ(outside[0], 0);
  EXPECT_EQ(outside[1], 0);
  EXPECT_EQ(outside[2], 0);
}

TEST(OverlayTest, DrawSegmentationsMultipleMasksDistinctColors) {
  cv::Mat img = makeBlankImage();
  perception_msgs::msg::SegmentationArray arr;
  arr.segmentations.push_back(
      makeSegmentation(1, img.rows, img.cols, cv::Rect(50, 50, 40, 40)));
  arr.segmentations.push_back(
      makeSegmentation(2, img.rows, img.cols, cv::Rect(300, 300, 40, 40)));
  EXPECT_EQ(draw_segmentations(img, arr, 0.7), 0u);

  const auto p1 = img.at<cv::Vec3b>(60, 60);
  const auto p2 = img.at<cv::Vec3b>(310, 310);
  const int sum1 = p1[0] + p1[1] + p1[2];
  const int sum2 = p2[0] + p2[1] + p2[2];
  EXPECT_GT(sum1, 0);
  EXPECT_GT(sum2, 0);
  // Distinct ids → distinct hues → at least one channel should differ.
  const bool differ = (p1[0] != p2[0]) || (p1[1] != p2[1]) || (p1[2] != p2[2]);
  EXPECT_TRUE(differ) << "Different ids should map to different colors";
}

TEST(OverlayTest, DrawSegmentationsClampsAlphaOutsideRange) {
  cv::Mat img_low = makeBlankImage();
  cv::Mat img_high = makeBlankImage();
  perception_msgs::msg::SegmentationArray arr;
  arr.segmentations.push_back(
      makeSegmentation(5, img_low.rows, img_low.cols,
                       cv::Rect(100, 100, 30, 30)));

  // alpha < 0 → clamped to 0 → image stays (mostly) unchanged in mask ROI.
  EXPECT_EQ(draw_segmentations(img_low, arr, -0.5), 0u);
  const auto inside_low = img_low.at<cv::Vec3b>(115, 115);
  EXPECT_EQ(inside_low[0] + inside_low[1] + inside_low[2], 0)
      << "alpha clamped to 0 should leave pixels black";

  // alpha > 1 → clamped to 1 → full color replacement on foreground.
  EXPECT_EQ(draw_segmentations(img_high, arr, 1.7), 0u);
  const auto inside_high = img_high.at<cv::Vec3b>(115, 115);
  EXPECT_GT(inside_high[0] + inside_high[1] + inside_high[2], 0);
}
