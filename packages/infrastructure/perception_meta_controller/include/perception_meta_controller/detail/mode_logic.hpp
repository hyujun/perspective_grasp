// Copyright 2026 perspective_grasp
// SPDX-License-Identifier: Apache-2.0

#pragma once

#include <rclcpp/time.hpp>

#include <perception_msgs/msg/associated_pose_array.hpp>

#include <set>
#include <string>
#include <unordered_map>
#include <vector>

namespace perspective_grasp::detail {

/// Returns true if `mode` is one of the modes the meta controller
/// understands (NORMAL, HIGH_PRECISION, SCENE_ANALYSIS).
bool is_valid_mode(const std::string& mode);

/// Compute the active perception node list for the given pipeline mode.
/// When `num_cameras > 1`, the cross_camera_associator is appended.
/// Returns an empty vector if the mode is unknown.
std::vector<std::string> compute_active_nodes(const std::string& mode,
                                              int num_cameras);

/// Per-object camera visibility record.
struct ObjectVisibility {
  std::set<int> camera_ids;
  rclcpp::Time last_seen;
};

struct VisibilityCounts {
  int tracked = 0;
  int lost = 0;
};

/// Fold one AssociatedPoseArray message into the visibility map,
/// prune entries older than `stale_timeout_sec`, and return tracked/lost
/// counts across the surviving map.
VisibilityCounts update_visibility(
    std::unordered_map<int, ObjectVisibility>& visibility,
    const perception_msgs::msg::AssociatedPoseArray& msg,
    const rclcpp::Time& now, double stale_timeout_sec);

}  // namespace perspective_grasp::detail
