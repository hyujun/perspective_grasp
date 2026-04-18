#pragma once

#include <memory>
#include <string>
#include <unordered_map>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>
#include <diagnostic_msgs/msg/diagnostic_array.hpp>

#include <perception_msgs/msg/detection_array.hpp>
#include <perception_msgs/msg/pose_with_meta_array.hpp>

#include "yolo_pcl_cpp_tracker/cad_model_manager.hpp"
#include "yolo_pcl_cpp_tracker/object_tracker.hpp"
#include "teaser_icp_hybrid/hybrid_registrator.hpp"

namespace perspective_grasp {

class PclIcpPoseEstimator : public rclcpp::Node {
 public:
  explicit PclIcpPoseEstimator(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

 private:
  /// Synchronized callback for detections + point cloud
  void syncCallback(
      const perception_msgs::msg::DetectionArray::ConstSharedPtr& dets,
      const sensor_msgs::msg::PointCloud2::ConstSharedPtr& cloud);

  /// Camera info callback (for 3D projection)
  void cameraInfoCallback(
      const sensor_msgs::msg::CameraInfo::ConstSharedPtr& msg);

  /// Crop point cloud to a 2D bounding box region
  pcl::PointCloud<pcl::PointXYZ>::Ptr cropToRoi(
      const sensor_msgs::msg::PointCloud2::ConstSharedPtr& cloud,
      const sensor_msgs::msg::RegionOfInterest& roi) const;

  /// Preprocess point cloud: voxel downsample + statistical outlier removal
  pcl::PointCloud<pcl::PointXYZ>::Ptr preprocess(
      const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud) const;

  /// Run ICP (or TEASER+ICP) for a single object and return pose
  void estimatePose(int object_id, const std::string& class_name,
                    const pcl::PointCloud<pcl::PointXYZ>::Ptr& scene_crop);

  /// Publish results and broadcast TF
  void publishResults();

  // Message filter subscriptions
  message_filters::Subscriber<perception_msgs::msg::DetectionArray> det_sub_;
  message_filters::Subscriber<sensor_msgs::msg::PointCloud2> cloud_sub_;

  using SyncPolicy = message_filters::sync_policies::ApproximateTime<
      perception_msgs::msg::DetectionArray,
      sensor_msgs::msg::PointCloud2>;
  std::shared_ptr<message_filters::Synchronizer<SyncPolicy>> sync_;

  // Camera info subscription
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_sub_;

  // Publishers
  rclcpp::Publisher<perception_msgs::msg::PoseWithMetaArray>::SharedPtr pose_pub_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diag_pub_;

  // TF broadcaster
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

  // Per-object trackers
  std::unordered_map<int, ObjectTracker> trackers_;

  // CAD model manager
  CadModelManager cad_manager_;

  // Registration pipeline
  std::unique_ptr<HybridRegistrator> registrator_;

  // Parameters
  double voxel_leaf_size_{0.003};
  int outlier_mean_k_{50};
  double outlier_stddev_{1.0};
  int max_lost_frames_{15};
  std::string camera_frame_id_{"camera_color_optical_frame"};

  // Latest camera info
  bool has_camera_info_{false};
  int image_width_{0};
  int image_height_{0};

  // Latest results for publishing
  perception_msgs::msg::PoseWithMetaArray latest_results_;
};

}  // namespace perspective_grasp
