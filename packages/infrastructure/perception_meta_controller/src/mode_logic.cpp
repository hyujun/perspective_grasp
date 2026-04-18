// Copyright 2026 perspective_grasp
// SPDX-License-Identifier: Apache-2.0

#include "perception_meta_controller/detail/mode_logic.hpp"

namespace perspective_grasp::detail {

bool is_valid_mode(const std::string& mode) {
  return mode == "NORMAL" || mode == "HIGH_PRECISION" ||
         mode == "SCENE_ANALYSIS";
}

std::vector<std::string> compute_active_nodes(const std::string& mode,
                                              int num_cameras) {
  std::vector<std::string> nodes;
  if (mode == "NORMAL") {
    nodes = {"yolo_byte_tracker", "pcl_icp_pose_estimator", "pose_filter"};
  } else if (mode == "HIGH_PRECISION") {
    nodes = {"yolo_byte_tracker",     "pcl_icp_pose_estimator",
             "foundationpose_tracker", "pose_filter",
             "pose_graph_smoother"};
  } else if (mode == "SCENE_ANALYSIS") {
    nodes = {"yolo_byte_tracker", "sam2_segmentor", "cosypose_optimizer",
             "pose_filter"};
  } else {
    return {};
  }

  if (num_cameras > 1) {
    nodes.emplace_back("cross_camera_associator");
  }
  return nodes;
}

VisibilityCounts update_visibility(
    std::unordered_map<int, ObjectVisibility>& visibility,
    const perception_msgs::msg::AssociatedPoseArray& msg,
    const rclcpp::Time& now, double stale_timeout_sec) {
  for (const auto& ap : msg.poses) {
    auto& vis = visibility[ap.global_id];
    vis.camera_ids.clear();
    for (auto cam_id : ap.source_camera_ids) {
      vis.camera_ids.insert(cam_id);
    }
    vis.last_seen = now;
  }

  VisibilityCounts counts;
  for (auto it = visibility.begin(); it != visibility.end();) {
    const double age = (now - it->second.last_seen).seconds();
    if (age > stale_timeout_sec) {
      it = visibility.erase(it);
    } else {
      if (it->second.camera_ids.empty()) {
        ++counts.lost;
      } else {
        ++counts.tracked;
      }
      ++it;
    }
  }
  return counts;
}

}  // namespace perspective_grasp::detail
