// Copyright 2026 perspective_grasp
// SPDX-License-Identifier: Apache-2.0

#include "perception_meta_controller/detail/mode_logic.hpp"

#include <gtest/gtest.h>

#include <rclcpp/time.hpp>

#include <perception_msgs/msg/associated_pose.hpp>
#include <perception_msgs/msg/associated_pose_array.hpp>

#include <algorithm>
#include <chrono>
#include <string>
#include <unordered_map>

namespace {

using perspective_grasp::detail::ObjectVisibility;
using perspective_grasp::detail::VisibilityCounts;
using perspective_grasp::detail::compute_active_nodes;
using perspective_grasp::detail::is_valid_mode;
using perspective_grasp::detail::update_visibility;

bool contains(const std::vector<std::string>& v, const std::string& s) {
  return std::find(v.begin(), v.end(), s) != v.end();
}

perception_msgs::msg::AssociatedPose makeObs(int global_id,
                                             std::vector<int> cam_ids) {
  perception_msgs::msg::AssociatedPose ap;
  ap.global_id = global_id;
  ap.source_camera_ids = std::move(cam_ids);
  ap.num_observing_cameras =
      static_cast<int>(ap.source_camera_ids.size());
  return ap;
}

}  // namespace

// --- is_valid_mode -----------------------------------------------------------

TEST(ModeLogicTest, ValidModesAccepted) {
  EXPECT_TRUE(is_valid_mode("NORMAL"));
  EXPECT_TRUE(is_valid_mode("HIGH_PRECISION"));
  EXPECT_TRUE(is_valid_mode("SCENE_ANALYSIS"));
}

TEST(ModeLogicTest, InvalidModesRejected) {
  EXPECT_FALSE(is_valid_mode(""));
  EXPECT_FALSE(is_valid_mode("normal"));  // case-sensitive
  EXPECT_FALSE(is_valid_mode("UNKNOWN"));
  EXPECT_FALSE(is_valid_mode("NORMAL  "));
}

// --- compute_active_nodes ----------------------------------------------------

TEST(ModeLogicTest, NormalModeBaselineNodes) {
  auto nodes = compute_active_nodes("NORMAL", 1);
  EXPECT_TRUE(contains(nodes, "yolo_byte_tracker"));
  EXPECT_TRUE(contains(nodes, "pcl_icp_pose_estimator"));
  EXPECT_TRUE(contains(nodes, "pose_filter"));
  EXPECT_FALSE(contains(nodes, "foundationpose_tracker"));
  EXPECT_FALSE(contains(nodes, "sam2_segmentor"));
  EXPECT_FALSE(contains(nodes, "cross_camera_associator"));
}

TEST(ModeLogicTest, HighPrecisionAddsRefinement) {
  auto nodes = compute_active_nodes("HIGH_PRECISION", 1);
  EXPECT_TRUE(contains(nodes, "yolo_byte_tracker"));
  EXPECT_TRUE(contains(nodes, "foundationpose_tracker"));
  EXPECT_TRUE(contains(nodes, "pose_graph_smoother"));
}

TEST(ModeLogicTest, SceneAnalysisUsesSam2Cosypose) {
  auto nodes = compute_active_nodes("SCENE_ANALYSIS", 1);
  EXPECT_TRUE(contains(nodes, "sam2_segmentor"));
  EXPECT_TRUE(contains(nodes, "cosypose_optimizer"));
  // Scene analysis mode does NOT use ICP-based pose estimator.
  EXPECT_FALSE(contains(nodes, "pcl_icp_pose_estimator"));
}

TEST(ModeLogicTest, UnknownModeReturnsEmpty) {
  EXPECT_TRUE(compute_active_nodes("BOGUS", 1).empty());
  EXPECT_TRUE(compute_active_nodes("", 3).empty());
}

TEST(ModeLogicTest, MultiCameraAddsAssociatorAcrossAllModes) {
  for (const std::string mode :
       {"NORMAL", "HIGH_PRECISION", "SCENE_ANALYSIS"}) {
    auto single = compute_active_nodes(mode, 1);
    auto multi = compute_active_nodes(mode, 2);
    auto triple = compute_active_nodes(mode, 3);

    EXPECT_FALSE(contains(single, "cross_camera_associator")) << mode;
    EXPECT_TRUE(contains(multi, "cross_camera_associator")) << mode;
    EXPECT_TRUE(contains(triple, "cross_camera_associator")) << mode;
    EXPECT_EQ(multi.size(), single.size() + 1U) << mode;
  }
}

TEST(ModeLogicTest, ZeroOrNegativeCamerasNoAssociator) {
  auto nodes0 = compute_active_nodes("NORMAL", 0);
  auto nodes_neg = compute_active_nodes("NORMAL", -1);
  EXPECT_FALSE(contains(nodes0, "cross_camera_associator"));
  EXPECT_FALSE(contains(nodes_neg, "cross_camera_associator"));
}

// --- update_visibility -------------------------------------------------------

TEST(ModeLogicTest, EmptyMessageEmptyMap) {
  std::unordered_map<int, ObjectVisibility> visibility;
  perception_msgs::msg::AssociatedPoseArray msg;
  rclcpp::Time now(1'000'000'000);

  auto counts = update_visibility(visibility, msg, now, 3.0);
  EXPECT_EQ(counts.tracked, 0);
  EXPECT_EQ(counts.lost, 0);
  EXPECT_TRUE(visibility.empty());
}

TEST(ModeLogicTest, NewObservationCountedAsTracked) {
  std::unordered_map<int, ObjectVisibility> visibility;
  perception_msgs::msg::AssociatedPoseArray msg;
  msg.poses.push_back(makeObs(42, {0, 1}));

  rclcpp::Time now(2'000'000'000);
  auto counts = update_visibility(visibility, msg, now, 3.0);

  EXPECT_EQ(counts.tracked, 1);
  EXPECT_EQ(counts.lost, 0);
  ASSERT_EQ(visibility.count(42), 1U);
  EXPECT_EQ(visibility[42].camera_ids.size(), 2U);
  EXPECT_EQ(visibility[42].last_seen.nanoseconds(), now.nanoseconds());
}

TEST(ModeLogicTest, ObservationWithEmptyCameraListCountsAsLost) {
  std::unordered_map<int, ObjectVisibility> visibility;
  perception_msgs::msg::AssociatedPoseArray msg;
  msg.poses.push_back(makeObs(7, {}));

  rclcpp::Time now(1'000'000'000);
  auto counts = update_visibility(visibility, msg, now, 3.0);

  EXPECT_EQ(counts.tracked, 0);
  EXPECT_EQ(counts.lost, 1);
}

TEST(ModeLogicTest, StaleEntriesPruned) {
  std::unordered_map<int, ObjectVisibility> visibility;
  // Pre-populate with an entry last seen at t=0.
  ObjectVisibility old;
  old.camera_ids.insert(0);
  old.last_seen = rclcpp::Time(0);
  visibility[99] = old;

  // Now is t=5s, timeout=3s → entry is stale
  perception_msgs::msg::AssociatedPoseArray msg;
  rclcpp::Time now(5'000'000'000);

  auto counts = update_visibility(visibility, msg, now, 3.0);
  EXPECT_EQ(counts.tracked, 0);
  EXPECT_EQ(counts.lost, 0);
  EXPECT_EQ(visibility.count(99), 0U);
}

TEST(ModeLogicTest, RefreshingEntryPreventsPrune) {
  std::unordered_map<int, ObjectVisibility> visibility;
  ObjectVisibility old;
  old.camera_ids.insert(0);
  old.last_seen = rclcpp::Time(0);
  visibility[55] = old;

  perception_msgs::msg::AssociatedPoseArray msg;
  msg.poses.push_back(makeObs(55, {1, 2}));

  rclcpp::Time now(5'000'000'000);  // 5s later (> timeout)
  auto counts = update_visibility(visibility, msg, now, 3.0);

  EXPECT_EQ(counts.tracked, 1) << "Refresh must keep the entry alive";
  ASSERT_EQ(visibility.count(55), 1U);
  // camera_ids must have been replaced with the new list {1, 2}
  EXPECT_EQ(visibility[55].camera_ids.size(), 2U);
  EXPECT_EQ(visibility[55].camera_ids.count(0), 0U);
  EXPECT_EQ(visibility[55].camera_ids.count(1), 1U);
}

TEST(ModeLogicTest, MixedTrackedLostAndStale) {
  std::unordered_map<int, ObjectVisibility> visibility;

  // Stale entry, will be pruned
  ObjectVisibility stale;
  stale.camera_ids.insert(0);
  stale.last_seen = rclcpp::Time(0);
  visibility[1] = stale;

  // Fresh entry without any cameras → lost
  ObjectVisibility lost;
  lost.last_seen = rclcpp::Time(4'500'000'000);
  visibility[2] = lost;

  // New observation → tracked
  perception_msgs::msg::AssociatedPoseArray msg;
  msg.poses.push_back(makeObs(3, {0}));

  rclcpp::Time now(5'000'000'000);
  auto counts = update_visibility(visibility, msg, now, 3.0);

  EXPECT_EQ(counts.tracked, 1);
  EXPECT_EQ(counts.lost, 1);
  EXPECT_EQ(visibility.count(1), 0U) << "stale entry should be pruned";
  EXPECT_EQ(visibility.count(2), 1U);
  EXPECT_EQ(visibility.count(3), 1U);
}
