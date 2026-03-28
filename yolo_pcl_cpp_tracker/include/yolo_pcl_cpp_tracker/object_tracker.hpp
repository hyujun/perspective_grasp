#pragma once

#include <string>

#include <Eigen/Geometry>

namespace perspective_grasp {

/// Per-object tracking state
struct ObjectTracker {
  int id{-1};
  std::string class_name;
  Eigen::Isometry3d last_pose{Eigen::Isometry3d::Identity()};
  double last_fitness{1.0};
  int lost_count{0};
  bool initialized{false};

  static constexpr int kMaxLostFrames = 15;
  static constexpr double kFitnessThreshold = 0.001;

  /// Returns true if global re-initialization (TEASER++) is needed
  bool needsGlobalReInit() const {
    return !initialized || lost_count > kMaxLostFrames ||
           last_fitness > kFitnessThreshold;
  }

  /// Mark as successfully tracked this frame
  void markTracked(const Eigen::Isometry3d& pose, double fitness) {
    last_pose = pose;
    last_fitness = fitness;
    lost_count = 0;
    initialized = true;
  }

  /// Mark as lost this frame (detection missing)
  void markLost() { ++lost_count; }

  /// Check if this tracker should be pruned
  bool isStale() const { return lost_count > kMaxLostFrames * 3; }
};

}  // namespace perspective_grasp
