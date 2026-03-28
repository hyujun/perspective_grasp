#include "cross_camera_associator/camera_pose_buffer.hpp"

namespace perspective_grasp {

void CameraPoseBuffer::update(
    int cam_id,
    perception_msgs::msg::PoseWithMetaArray::ConstSharedPtr msg) {
  std::lock_guard<std::mutex> lock(mutex_);
  rclcpp::Time stamp(msg->header.stamp);
  buffers_[cam_id] = {msg, stamp};
}

std::vector<TimedDetections> CameraPoseBuffer::snapshot(
    rclcpp::Time now, double max_age_ms) const {
  std::lock_guard<std::mutex> lock(mutex_);
  std::vector<TimedDetections> result;

  const double max_age_sec = max_age_ms / 1000.0;

  for (const auto& [cam_id, entry] : buffers_) {
    double age = (now - entry.stamp).seconds();
    if (age <= max_age_sec && age >= 0.0) {
      result.push_back({cam_id, entry.msg, entry.stamp});
    }
  }
  return result;
}

}  // namespace perspective_grasp
