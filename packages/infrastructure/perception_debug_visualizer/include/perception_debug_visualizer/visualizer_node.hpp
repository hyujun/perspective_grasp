#pragma once

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.hpp>
#include <image_transport/image_transport.hpp>
#include <perception_msgs/msg/detection_array.hpp>
#include <perception_msgs/msg/pose_with_meta_array.hpp>
#include <perception_msgs/msg/pipeline_status.hpp>
#include <perception_msgs/msg/segmentation_array.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <opencv2/opencv.hpp>

#include <array>
#include <memory>
#include <optional>
#include <string>
#include <vector>

namespace perspective_grasp
{

class VisualizerNode : public rclcpp::Node
{
public:
  explicit VisualizerNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

  // Compose "/ns/suffix" (or "/suffix" when ns is empty). Exposed for tests.
  static std::string compose_topic(const std::string & ns, const std::string & suffix);

private:
  struct CameraIntrinsics {
    std::array<double, 9> k{};
    std::string frame_id;
  };

  void image_callback(
    std::size_t cam_index, const sensor_msgs::msg::Image::ConstSharedPtr & msg);
  void detections_callback(
    std::size_t cam_index, const perception_msgs::msg::DetectionArray::SharedPtr msg);
  void masks_callback(
    std::size_t cam_index, const perception_msgs::msg::SegmentationArray::SharedPtr msg);
  void camera_info_callback(
    std::size_t cam_index, const sensor_msgs::msg::CameraInfo::SharedPtr msg);
  void smoothed_poses_callback(const perception_msgs::msg::PoseWithMetaArray::SharedPtr msg);
  void pipeline_status_callback(const perception_msgs::msg::PipelineStatus::SharedPtr msg);

  // Overlay the pose-axis triads for `latest_poses_` onto `image` in the
  // optical frame of camera `cam_index`. No-op if intrinsics / TF / axes are
  // disabled or unavailable. Separated so `image_callback` stays short.
  void overlay_pose_axes(std::size_t cam_index, cv::Mat & image);

  rcl_interfaces::msg::SetParametersResult on_set_parameters(
    const std::vector<rclcpp::Parameter> & params);

  // Per-camera state
  std::vector<std::string> camera_namespaces_;
  std::vector<rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr> sub_images_;
  std::vector<rclcpp::Subscription<perception_msgs::msg::DetectionArray>::SharedPtr>
    sub_detections_;
  std::vector<rclcpp::Subscription<perception_msgs::msg::SegmentationArray>::SharedPtr>
    sub_masks_;
  std::vector<rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr>
    sub_camera_info_;
  std::vector<perception_msgs::msg::DetectionArray::SharedPtr> latest_detections_;
  std::vector<perception_msgs::msg::SegmentationArray::SharedPtr> latest_masks_;
  std::vector<std::optional<CameraIntrinsics>> latest_intrinsics_;

  // Global subs
  rclcpp::Subscription<perception_msgs::msg::PoseWithMetaArray>::SharedPtr sub_smoothed_;
  rclcpp::Subscription<perception_msgs::msg::PipelineStatus>::SharedPtr sub_status_;

  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_debug_image_;
  OnSetParametersCallbackHandle::SharedPtr param_cb_handle_;

  // TF lookup for projecting smoothed poses into the active camera's optical
  // frame. Buffer+listener are owned here; all lookups happen on the image
  // callback thread.
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  // Cached state
  perception_msgs::msg::PoseWithMetaArray::SharedPtr latest_poses_;
  std::string pipeline_mode_;
  std::size_t active_camera_index_ {0};

  // Overlay toggles (hot-settable via on_set_parameters).
  bool enable_sam2_masks_ {true};
  float mask_alpha_ {0.4f};
  bool enable_pose_axes_ {true};
  double axis_length_m_ {0.05};
  int pose_axis_thickness_ {2};
};

}  // namespace perspective_grasp
