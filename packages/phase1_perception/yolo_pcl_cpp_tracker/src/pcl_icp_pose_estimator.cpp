#include "yolo_pcl_cpp_tracker/pcl_icp_pose_estimator.hpp"

#include <unordered_set>

#include <pcl_conversions/pcl_conversions.h>
#include <tf2_eigen/tf2_eigen.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>

#include "yolo_pcl_cpp_tracker/pcl_utils.hpp"

namespace perspective_grasp {

PclIcpPoseEstimator::PclIcpPoseEstimator(const rclcpp::NodeOptions& options)
    : Node("pcl_icp_pose_estimator", options) {
  // Declare parameters
  this->declare_parameter("detection_topic", "yolo/detections");
  this->declare_parameter("cloud_topic", "/camera/depth/color/points");
  this->declare_parameter("camera_info_topic", "/camera/depth/camera_info");
  this->declare_parameter("voxel_leaf_size", 0.003);
  this->declare_parameter("outlier_mean_k", 50);
  this->declare_parameter("outlier_stddev", 1.0);
  this->declare_parameter("max_lost_frames", 15);
  this->declare_parameter("model_dir", "");
  this->declare_parameter("camera_frame_id", "camera_color_optical_frame");
  this->declare_parameter("icp_max_iterations", 50);
  this->declare_parameter("icp_transformation_epsilon", 1e-8);
  this->declare_parameter("icp_fitness_epsilon", 1e-6);
  this->declare_parameter("fitness_threshold", 0.001);

  // Get parameters
  auto det_topic = this->get_parameter("detection_topic").as_string();
  auto cloud_topic = this->get_parameter("cloud_topic").as_string();
  auto camera_info_topic = this->get_parameter("camera_info_topic").as_string();
  voxel_leaf_size_ = this->get_parameter("voxel_leaf_size").as_double();
  outlier_mean_k_ = this->get_parameter("outlier_mean_k").as_int();
  outlier_stddev_ = this->get_parameter("outlier_stddev").as_double();
  max_lost_frames_ = this->get_parameter("max_lost_frames").as_int();
  camera_frame_id_ = this->get_parameter("camera_frame_id").as_string();
  auto model_dir = this->get_parameter("model_dir").as_string();

  // Load CAD models
  if (!model_dir.empty()) {
    cad_manager_.loadModels(model_dir);
    RCLCPP_INFO(this->get_logger(), "Loaded %zu CAD models from %s",
                cad_manager_.modelCount(), model_dir.c_str());
  } else {
    RCLCPP_WARN(this->get_logger(), "No model_dir specified - ICP will not work");
  }

  // Configure registration pipeline
  RegistrationConfig reg_config;
  reg_config.icp_max_iterations =
      this->get_parameter("icp_max_iterations").as_int();
  reg_config.icp_transformation_epsilon =
      this->get_parameter("icp_transformation_epsilon").as_double();
  reg_config.icp_euclidean_fitness_epsilon =
      this->get_parameter("icp_fitness_epsilon").as_double();
  reg_config.voxel_size = voxel_leaf_size_;
  registrator_ = std::make_unique<HybridRegistrator>(reg_config);

  // QoS: best effort for sensor data
  auto sensor_qos = rclcpp::SensorDataQoS();

  // Message filter subscribers
  det_sub_.subscribe(this, det_topic, sensor_qos.get_rmw_qos_profile());
  cloud_sub_.subscribe(this, cloud_topic, sensor_qos.get_rmw_qos_profile());

  // Approximate time synchronizer (30ms slop)
  sync_ = std::make_shared<message_filters::Synchronizer<SyncPolicy>>(
      SyncPolicy(10), det_sub_, cloud_sub_);
  sync_->setMaxIntervalDuration(rclcpp::Duration::from_seconds(0.03));
  sync_->registerCallback(
      std::bind(&PclIcpPoseEstimator::syncCallback, this,
                std::placeholders::_1, std::placeholders::_2));

  // Camera info subscriber
  camera_info_sub_ = this->create_subscription<sensor_msgs::msg::CameraInfo>(
      camera_info_topic, sensor_qos,
      std::bind(&PclIcpPoseEstimator::cameraInfoCallback, this,
                std::placeholders::_1));

  // Publishers
  pose_pub_ = this->create_publisher<perception_msgs::msg::PoseWithMetaArray>(
      "yolo_tracker/raw_poses", rclcpp::QoS(1).best_effort());

  diag_pub_ = this->create_publisher<diagnostic_msgs::msg::DiagnosticArray>(
      "yolo_tracker/diagnostics", rclcpp::QoS(1).best_effort());

  // TF broadcaster
  tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

  RCLCPP_INFO(this->get_logger(), "PclIcpPoseEstimator initialized");
}

void PclIcpPoseEstimator::cameraInfoCallback(
    const sensor_msgs::msg::CameraInfo::ConstSharedPtr& msg) {
  image_width_ = static_cast<int>(msg->width);
  image_height_ = static_cast<int>(msg->height);
  has_camera_info_ = true;
}

void PclIcpPoseEstimator::syncCallback(
    const perception_msgs::msg::DetectionArray::ConstSharedPtr& dets,
    const sensor_msgs::msg::PointCloud2::ConstSharedPtr& cloud) {
  if (!has_camera_info_) {
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                         "Waiting for camera_info...");
    return;
  }

  // Track which IDs are seen this frame
  std::unordered_set<int> seen_ids;

  // Prepare output message
  latest_results_.header = dets->header;
  latest_results_.poses.clear();

  // Process each detection
  for (const auto& det : dets->detections) {
    seen_ids.insert(det.id);

    // Crop point cloud to detection ROI
    pcl::PointCloud<pcl::PointXYZ> full_cloud;
    pcl::fromROSMsg(*cloud, full_cloud);
    auto scene_crop = pcl_utils::cropToRoi(full_cloud, det.bbox);
    if (!scene_crop || scene_crop->size() < 50) {
      continue;
    }

    // Preprocess (voxel grid + outlier removal)
    auto preprocessed = pcl_utils::preprocess(
        scene_crop, voxel_leaf_size_, outlier_mean_k_, outlier_stddev_);
    if (!preprocessed || preprocessed->empty()) {
      continue;
    }

    // Estimate pose
    estimatePose(det.id, det.class_name, preprocessed);
  }

  // Mark unseen trackers as lost
  for (auto& [id, tracker] : trackers_) {
    if (seen_ids.find(id) == seen_ids.end()) {
      tracker.markLost();
    }
  }

  // Prune stale trackers
  for (auto it = trackers_.begin(); it != trackers_.end();) {
    if (it->second.isStale()) {
      it = trackers_.erase(it);
    } else {
      ++it;
    }
  }

  // Publish results and broadcast TF
  publishResults();
}

void PclIcpPoseEstimator::estimatePose(
    int object_id, const std::string& class_name,
    const pcl::PointCloud<pcl::PointXYZ>::Ptr& scene_crop) {
  // Get or create tracker for this object
  auto& tracker = trackers_[object_id];
  if (tracker.id < 0) {
    tracker.id = object_id;
    tracker.class_name = class_name;
  }

  // Get CAD model
  auto cad_model = cad_manager_.getModel(class_name);
  if (!cad_model) {
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                         "No CAD model for class '%s'", class_name.c_str());
    return;
  }

  RegistrationResult result;
  if (tracker.needsGlobalReInit()) {
    // Full TEASER++ + ICP pipeline for initial registration or re-init
    result = registrator_->align(cad_model, scene_crop);
  } else {
    // ICP-only with previous pose as initial guess
    result = registrator_->refineIcp(cad_model, scene_crop, tracker.last_pose);
  }

  if (result.success) {
    tracker.markTracked(result.transform, result.icp_fitness);

    // Build output message
    perception_msgs::msg::PoseWithMeta pose_msg;
    pose_msg.object_id = object_id;
    pose_msg.class_name = class_name;
    pose_msg.source = "yolo_tracker";
    pose_msg.confidence = static_cast<float>(1.0 - result.icp_fitness);
    pose_msg.fitness_score = static_cast<float>(result.icp_fitness);
    pose_msg.pose.header.stamp = this->now();
    pose_msg.pose.header.frame_id = camera_frame_id_;
    pose_msg.pose.pose = tf2::toMsg(result.transform);

    latest_results_.poses.push_back(pose_msg);
  } else {
    tracker.markLost();
  }
}

void PclIcpPoseEstimator::publishResults() {
  if (!latest_results_.poses.empty()) {
    latest_results_.header.stamp = this->now();
    pose_pub_->publish(latest_results_);
  }

  // Broadcast TF for each tracked object
  for (const auto& pose_msg : latest_results_.poses) {
    geometry_msgs::msg::TransformStamped tf;
    tf.header = pose_msg.pose.header;
    tf.child_frame_id = "object_" + std::to_string(pose_msg.object_id);
    tf.transform.translation.x = pose_msg.pose.pose.position.x;
    tf.transform.translation.y = pose_msg.pose.pose.position.y;
    tf.transform.translation.z = pose_msg.pose.pose.position.z;
    tf.transform.rotation = pose_msg.pose.pose.orientation;
    tf_broadcaster_->sendTransform(tf);
  }

  // Publish diagnostics
  diagnostic_msgs::msg::DiagnosticArray diag_msg;
  diag_msg.header.stamp = this->now();
  for (const auto& [id, tracker] : trackers_) {
    diagnostic_msgs::msg::DiagnosticStatus status;
    status.name = "object_" + std::to_string(id);
    status.level = tracker.initialized
                       ? diagnostic_msgs::msg::DiagnosticStatus::OK
                       : diagnostic_msgs::msg::DiagnosticStatus::WARN;
    status.message = tracker.initialized ? "tracking" : "lost";

    diagnostic_msgs::msg::KeyValue fitness_kv;
    fitness_kv.key = "fitness";
    fitness_kv.value = std::to_string(tracker.last_fitness);
    status.values.push_back(fitness_kv);

    diagnostic_msgs::msg::KeyValue lost_kv;
    lost_kv.key = "lost_count";
    lost_kv.value = std::to_string(tracker.lost_count);
    status.values.push_back(lost_kv);

    diag_msg.status.push_back(status);
  }
  diag_pub_->publish(diag_msg);
}

}  // namespace perspective_grasp

#include <rclcpp/rclcpp.hpp>
int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<perspective_grasp::PclIcpPoseEstimator>());
  rclcpp::shutdown();
  return 0;
}
