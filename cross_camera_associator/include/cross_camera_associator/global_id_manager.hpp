#pragma once

#include <string>
#include <unordered_map>

#include <Eigen/Dense>
#include <rclcpp/time.hpp>

namespace perspective_grasp {

/// Maintains temporal consistency of global object IDs across frames.
class GlobalIdManager {
 public:
  /// @param distance_threshold  Max distance (meters) to consider same object.
  explicit GlobalIdManager(double distance_threshold = 0.05);

  /// Find the closest previous global object within threshold and reuse its ID,
  /// or assign a new one.
  int getOrAssignId(const Eigen::Vector3d& center,
                    const std::string& class_name, rclcpp::Time now);

  /// Remove objects that have not been seen for longer than timeout_sec.
  void pruneStaleObjects(double timeout_sec, rclcpp::Time now);

  /// Reset all state (mainly for testing).
  void reset();

  /// Number of currently tracked global objects.
  std::size_t size() const { return objects_.size(); }

 private:
  struct GlobalObject {
    Eigen::Vector3d position;
    std::string class_name;
    rclcpp::Time last_seen;
  };

  double distance_threshold_;
  int next_id_{0};
  std::unordered_map<int, GlobalObject> objects_;
};

}  // namespace perspective_grasp
