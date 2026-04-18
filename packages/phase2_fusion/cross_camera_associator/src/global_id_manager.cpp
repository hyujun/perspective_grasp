#include "cross_camera_associator/global_id_manager.hpp"

#include <cmath>
#include <limits>
#include <vector>

namespace perspective_grasp {

GlobalIdManager::GlobalIdManager(double distance_threshold)
    : distance_threshold_(distance_threshold) {}

int GlobalIdManager::getOrAssignId(const Eigen::Vector3d& center,
                                   const std::string& class_name,
                                   rclcpp::Time now) {
  int best_id = -1;
  double best_dist = std::numeric_limits<double>::max();

  for (const auto& [id, obj] : objects_) {
    if (obj.class_name != class_name) continue;
    double dist = (obj.position - center).norm();
    if (dist < best_dist) {
      best_dist = dist;
      best_id = id;
    }
  }

  if (best_id >= 0 && best_dist <= distance_threshold_) {
    // Update existing object
    auto& obj = objects_[best_id];
    obj.position = center;
    obj.last_seen = now;
    return best_id;
  }

  // Assign new ID
  int new_id = next_id_++;
  objects_[new_id] = {center, class_name, now};
  return new_id;
}

void GlobalIdManager::pruneStaleObjects(double timeout_sec, rclcpp::Time now) {
  std::vector<int> to_remove;
  for (const auto& [id, obj] : objects_) {
    double age = (now - obj.last_seen).seconds();
    if (age > timeout_sec) {
      to_remove.push_back(id);
    }
  }
  for (int id : to_remove) {
    objects_.erase(id);
  }
}

void GlobalIdManager::reset() {
  objects_.clear();
  next_id_ = 0;
}

}  // namespace perspective_grasp
