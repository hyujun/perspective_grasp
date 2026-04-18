#pragma once

#include <memory>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <diagnostic_msgs/msg/diagnostic_array.hpp>

#include <perception_msgs/msg/pose_with_meta_array.hpp>
#include <perception_msgs/msg/associated_pose_array.hpp>

#include "cross_camera_associator/camera_pose_buffer.hpp"
#include "cross_camera_associator/global_id_manager.hpp"
#include "cross_camera_associator/hungarian_solver.hpp"
#include "cross_camera_associator/union_find.hpp"

namespace perspective_grasp {

class AssociatorNode : public rclcpp::Node {
 public:
  explicit AssociatorNode(
      const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

 private:
  /// Timer callback that drives the association pipeline.
  void onAssociationTimer();

  /// Handle single-camera bypass: assign global IDs directly.
  void handleBypass(const TimedDetections& det);

  /// Handle multi-camera full association.
  void handleMultiCamera(const std::vector<TimedDetections>& detections);

  /// Transform a pose into base_frame. Returns false on failure.
  bool transformPose(const geometry_msgs::msg::PoseStamped& in,
                     geometry_msgs::msg::PoseStamped& out) const;

  /// Publish diagnostics summary.
  void publishDiagnostics(std::size_t num_cameras, std::size_t num_objects);

  // Parameters
  std::vector<std::string> camera_namespaces_;
  std::string base_frame_;
  double association_distance_threshold_;
  double lambda_rot_;
  double temporal_threshold_;
  double max_snapshot_age_ms_;
  double association_rate_hz_;
  double stale_object_timeout_sec_;

  // Core components
  CameraPoseBuffer buffer_;
  GlobalIdManager id_manager_;

  // TF2
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  // Subscriptions (one per camera)
  std::vector<rclcpp::Subscription<
      perception_msgs::msg::PoseWithMetaArray>::SharedPtr> subs_;

  // Publishers
  rclcpp::Publisher<perception_msgs::msg::AssociatedPoseArray>::SharedPtr
      pose_pub_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr
      diag_pub_;

  // Timer
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace perspective_grasp
