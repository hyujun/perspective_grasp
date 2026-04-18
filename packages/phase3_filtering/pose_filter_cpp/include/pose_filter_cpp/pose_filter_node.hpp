#pragma once

#include <memory>
#include <string>
#include <unordered_map>

#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <diagnostic_msgs/msg/diagnostic_array.hpp>

#include <perception_msgs/msg/pose_with_meta_array.hpp>
#include <perception_msgs/msg/pose_covariance_array.hpp>
#include <perception_msgs/msg/associated_pose_array.hpp>

#include "pose_filter_cpp/iekf_se3.hpp"

namespace perspective_grasp {

/// ROS 2 node that filters raw 6D poses from multiple perception sources.
/// Maintains one IekfSe3 instance per tracked object ID.
///
/// Supports two input modes:
///   - Associated mode (default): subscribes to /associated/poses from cross_camera_associator
///   - Legacy mode: subscribes to 4 hardcoded raw_poses topics (backward compat)
class PoseFilterNode : public rclcpp::Node {
 public:
  explicit PoseFilterNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

 private:
  // === Associated mode (multi-camera) ===
  void associatedPosesCallback(
      const perception_msgs::msg::AssociatedPoseArray::ConstSharedPtr& msg);
  rclcpp::Subscription<perception_msgs::msg::AssociatedPoseArray>::SharedPtr associated_sub_;

  // === Legacy mode (single-camera, backward compat) ===
  void rawPosesCallback(
      const perception_msgs::msg::PoseWithMetaArray::ConstSharedPtr& msg,
      const std::string& source_name, double noise_scale);
  rclcpp::Subscription<perception_msgs::msg::PoseWithMetaArray>::SharedPtr yolo_sub_;
  rclcpp::Subscription<perception_msgs::msg::PoseWithMetaArray>::SharedPtr fp_sub_;
  rclcpp::Subscription<perception_msgs::msg::PoseWithMetaArray>::SharedPtr megapose_sub_;
  rclcpp::Subscription<perception_msgs::msg::PoseWithMetaArray>::SharedPtr bundlesdf_sub_;

  /// Publish filtered poses + covariance + TF at fixed rate
  void publishTimerCallback();

  /// Prune stale filters that haven't received updates
  void pruneStaleFilters();

  // Publishers
  rclcpp::Publisher<perception_msgs::msg::PoseWithMetaArray>::SharedPtr filtered_pub_;
  rclcpp::Publisher<perception_msgs::msg::PoseCovarianceArray>::SharedPtr cov_pub_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diag_pub_;

  // TF broadcaster
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

  // Timer for fixed-rate publishing
  rclcpp::TimerBase::SharedPtr pub_timer_;

  // Per-object IEKF filters
  struct FilterState {
    IekfSe3 filter;
    std::string class_name;
    rclcpp::Time last_update;
  };
  std::unordered_map<int, FilterState> filters_;

  // Config
  IekfSe3::Config filter_config_;
  std::unordered_map<std::string, double> source_weights_;
  double stale_timeout_sec_{2.0};
  std::string output_frame_id_{"camera_color_optical_frame"};
  bool use_associated_input_{true};
};

}  // namespace perspective_grasp
