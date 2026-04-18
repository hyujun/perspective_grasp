#pragma once

#include <mutex>
#include <unordered_map>
#include <vector>

#include <rclcpp/time.hpp>
#include <perception_msgs/msg/pose_with_meta_array.hpp>

namespace perspective_grasp {

/// Per-detection snapshot with camera ID and timestamp.
struct TimedDetections {
  int camera_id;
  perception_msgs::msg::PoseWithMetaArray::ConstSharedPtr msg;
  rclcpp::Time stamp;
};

/// Thread-safe per-camera buffer storing the latest PoseWithMetaArray message.
class CameraPoseBuffer {
 public:
  /// Store or update data for a given camera.
  void update(int cam_id,
              perception_msgs::msg::PoseWithMetaArray::ConstSharedPtr msg);

  /// Return data from all cameras within the age limit.
  /// @param now        Current time.
  /// @param max_age_ms Maximum age in milliseconds.
  std::vector<TimedDetections> snapshot(rclcpp::Time now,
                                        double max_age_ms) const;

 private:
  struct Entry {
    perception_msgs::msg::PoseWithMetaArray::ConstSharedPtr msg;
    rclcpp::Time stamp;
  };

  mutable std::mutex mutex_;
  std::unordered_map<int, Entry> buffers_;
};

}  // namespace perspective_grasp
