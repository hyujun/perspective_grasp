#include "perception_debug_visualizer/visualizer_node.hpp"

#include "perception_debug_visualizer/detail/overlay.hpp"

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <rcl_interfaces/msg/set_parameters_result.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2/exceptions.h>

#include <algorithm>
#include <cstddef>
#include <vector>

namespace perspective_grasp
{

namespace {
constexpr const char * kDefaultImageSuffix = "camera/color/image_raw";
constexpr const char * kDefaultDetectionSuffix = "yolo/detections";
constexpr const char * kDefaultMasksSuffix = "sam2/masks";
constexpr const char * kDefaultCameraInfoSuffix = "camera/color/camera_info";
constexpr const char * kDefaultSmootherTopic = "/smoother/smoothed_poses";
constexpr const char * kDefaultPipelineStatusTopic = "/meta_controller/active_pipeline";
constexpr const char * kDefaultDebugImageTopic = "/debug/image";
}  // namespace

std::string VisualizerNode::compose_topic(
  const std::string & ns, const std::string & suffix)
{
  // Strip leading/trailing slashes from ns and leading slash from suffix.
  std::string ns_clean = ns;
  while (!ns_clean.empty() && ns_clean.front() == '/') ns_clean.erase(0, 1);
  while (!ns_clean.empty() && ns_clean.back() == '/') ns_clean.pop_back();
  std::string tail = suffix;
  while (!tail.empty() && tail.front() == '/') tail.erase(0, 1);
  if (ns_clean.empty()) {
    return "/" + tail;
  }
  return "/" + ns_clean + "/" + tail;
}

VisualizerNode::VisualizerNode(const rclcpp::NodeOptions & options)
: rclcpp::Node("perception_debug_visualizer", options),
  pipeline_mode_("UNKNOWN")
{
  declare_parameter<std::vector<std::string>>(
    "camera_namespaces", std::vector<std::string>{""});
  declare_parameter<std::string>("image_topic_suffix", kDefaultImageSuffix);
  declare_parameter<std::string>("detection_topic_suffix", kDefaultDetectionSuffix);
  declare_parameter<std::string>("sam2_masks_suffix", kDefaultMasksSuffix);
  declare_parameter<std::string>("camera_info_suffix", kDefaultCameraInfoSuffix);
  declare_parameter<bool>("enable_sam2_masks", true);
  declare_parameter<double>("mask_alpha", 0.4);
  declare_parameter<bool>("enable_pose_axes", true);
  declare_parameter<double>("axis_length_m", 0.05);
  declare_parameter<int>("pose_axis_thickness", 2);
  declare_parameter<int>("active_camera_index", 0);
  // Legacy override: if non-empty, replaces camera 0's computed image topic.
  // Kept for backward compatibility with single-camera users that set this.
  declare_parameter<std::string>("image_topic", "");

  camera_namespaces_ = get_parameter("camera_namespaces").as_string_array();
  if (camera_namespaces_.empty()) {
    camera_namespaces_.emplace_back("");
  }

  const std::string image_suffix = get_parameter("image_topic_suffix").as_string();
  const std::string det_suffix = get_parameter("detection_topic_suffix").as_string();
  const std::string masks_suffix = get_parameter("sam2_masks_suffix").as_string();
  const std::string camera_info_suffix =
    get_parameter("camera_info_suffix").as_string();
  const std::string legacy_image_topic = get_parameter("image_topic").as_string();
  enable_sam2_masks_ = get_parameter("enable_sam2_masks").as_bool();
  mask_alpha_ = static_cast<float>(
    std::clamp(get_parameter("mask_alpha").as_double(), 0.0, 1.0));
  enable_pose_axes_ = get_parameter("enable_pose_axes").as_bool();
  axis_length_m_ = std::max(1e-4, get_parameter("axis_length_m").as_double());
  pose_axis_thickness_ = static_cast<int>(
    std::clamp<int64_t>(get_parameter("pose_axis_thickness").as_int(), 1, 10));
  const int64_t requested_active = get_parameter("active_camera_index").as_int();
  active_camera_index_ = static_cast<std::size_t>(std::max<int64_t>(0, requested_active));
  if (active_camera_index_ >= camera_namespaces_.size()) {
    active_camera_index_ = 0;
  }

  auto sensor_qos = rclcpp::QoS(1).best_effort();
  auto reliable_qos = rclcpp::QoS(1).reliable();

  sub_images_.reserve(camera_namespaces_.size());
  sub_detections_.reserve(camera_namespaces_.size());
  sub_masks_.reserve(camera_namespaces_.size());
  sub_camera_info_.reserve(camera_namespaces_.size());
  latest_detections_.assign(camera_namespaces_.size(), nullptr);
  latest_masks_.assign(camera_namespaces_.size(), nullptr);
  latest_intrinsics_.assign(camera_namespaces_.size(), std::nullopt);

  tf_buffer_ = std::make_unique<tf2_ros::Buffer>(get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  for (std::size_t i = 0; i < camera_namespaces_.size(); ++i) {
    const std::string & ns = camera_namespaces_[i];
    std::string image_topic = compose_topic(ns, image_suffix);
    const std::string det_topic = compose_topic(ns, det_suffix);
    const std::string masks_topic = compose_topic(ns, masks_suffix);
    const std::string info_topic = compose_topic(ns, camera_info_suffix);

    if (i == 0 && !legacy_image_topic.empty()) {
      RCLCPP_INFO(
        get_logger(),
        "'image_topic' parameter overrides camera 0 image topic to '%s' (legacy path)",
        legacy_image_topic.c_str());
      image_topic = legacy_image_topic;
    }

    RCLCPP_INFO(
      get_logger(),
      "cam[%zu] ns='%s' image='%s' detections='%s' masks='%s' info='%s'",
      i, ns.c_str(), image_topic.c_str(), det_topic.c_str(),
      masks_topic.c_str(), info_topic.c_str());

    sub_images_.push_back(
      create_subscription<sensor_msgs::msg::Image>(
        image_topic, sensor_qos,
        [this, i](sensor_msgs::msg::Image::ConstSharedPtr msg) {
          image_callback(i, msg);
        }));

    sub_detections_.push_back(
      create_subscription<perception_msgs::msg::DetectionArray>(
        det_topic, sensor_qos,
        [this, i](perception_msgs::msg::DetectionArray::SharedPtr msg) {
          detections_callback(i, msg);
        }));

    sub_masks_.push_back(
      create_subscription<perception_msgs::msg::SegmentationArray>(
        masks_topic, sensor_qos,
        [this, i](perception_msgs::msg::SegmentationArray::SharedPtr msg) {
          masks_callback(i, msg);
        }));

    sub_camera_info_.push_back(
      create_subscription<sensor_msgs::msg::CameraInfo>(
        info_topic, sensor_qos,
        [this, i](sensor_msgs::msg::CameraInfo::SharedPtr msg) {
          camera_info_callback(i, msg);
        }));
  }

  sub_smoothed_ = create_subscription<perception_msgs::msg::PoseWithMetaArray>(
    kDefaultSmootherTopic, sensor_qos,
    std::bind(&VisualizerNode::smoothed_poses_callback, this, std::placeholders::_1));

  sub_status_ = create_subscription<perception_msgs::msg::PipelineStatus>(
    kDefaultPipelineStatusTopic, reliable_qos,
    std::bind(&VisualizerNode::pipeline_status_callback, this, std::placeholders::_1));

  pub_debug_image_ = create_publisher<sensor_msgs::msg::Image>(
    kDefaultDebugImageTopic, rclcpp::QoS(1).best_effort());

  param_cb_handle_ = add_on_set_parameters_callback(
    [this](const std::vector<rclcpp::Parameter> & params) {
      return on_set_parameters(params);
    });

  RCLCPP_INFO(
    get_logger(),
    "VisualizerNode started with %zu camera(s), active=%zu",
    camera_namespaces_.size(), active_camera_index_);
}

void VisualizerNode::image_callback(
  std::size_t cam_index, const sensor_msgs::msg::Image::ConstSharedPtr & msg)
{
  if (cam_index != active_camera_index_) {
    // Non-active cameras still subscribe so the user can hot-swap via param;
    // we just don't republish them.
    return;
  }
  try {
    auto cv_ptr = cv_bridge::toCvCopy(msg, "bgr8");
    const std::string & ns = camera_namespaces_[cam_index];
    const std::string mode_label = ns.empty()
      ? pipeline_mode_
      : pipeline_mode_ + "  cam=" + ns;
    if (enable_sam2_masks_ && cam_index < latest_masks_.size() &&
        latest_masks_[cam_index]) {
      detail::draw_masks(cv_ptr->image, *latest_masks_[cam_index], mask_alpha_);
    }
    if (cam_index < latest_detections_.size() && latest_detections_[cam_index]) {
      detail::draw_detections(cv_ptr->image, *latest_detections_[cam_index]);
    }
    if (enable_pose_axes_) {
      overlay_pose_axes(cam_index, cv_ptr->image);
    }
    detail::draw_mode_overlay(cv_ptr->image, mode_label);
    pub_debug_image_->publish(*cv_ptr->toImageMsg());
  } catch (const cv_bridge::Exception & e) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "cv_bridge error: %s", e.what());
  }
}

void VisualizerNode::overlay_pose_axes(std::size_t cam_index, cv::Mat & image)
{
  if (cam_index >= latest_intrinsics_.size() || !latest_intrinsics_[cam_index]) {
    return;
  }
  if (!latest_poses_ || latest_poses_->poses.empty()) {
    return;
  }
  const auto & intrinsics = *latest_intrinsics_[cam_index];
  if (intrinsics.frame_id.empty()) {
    return;
  }
  const std::string source_frame = latest_poses_->header.frame_id;
  if (source_frame.empty()) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 5000,
      "smoothed_poses header has empty frame_id; skipping pose-axis overlay");
    return;
  }

  geometry_msgs::msg::TransformStamped tf_cam_from_world;
  try {
    tf_cam_from_world = tf_buffer_->lookupTransform(
      intrinsics.frame_id, source_frame,
      tf2::TimePointZero);
  } catch (const tf2::TransformException & e) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 5000,
      "TF lookup '%s' -> '%s' failed: %s",
      source_frame.c_str(), intrinsics.frame_id.c_str(), e.what());
    return;
  }

  std::vector<detail::ProjectedAxis> projected;
  projected.reserve(latest_poses_->poses.size());
  for (const auto & p : latest_poses_->poses) {
    geometry_msgs::msg::PoseStamped pose_world;
    pose_world.header.frame_id = source_frame;
    pose_world.pose = p.pose.pose;
    geometry_msgs::msg::PoseStamped pose_cam;
    tf2::doTransform(pose_world, pose_cam, tf_cam_from_world);

    auto proj = detail::project_pose_axes(
      pose_cam.pose, intrinsics.k, axis_length_m_, p.object_id);
    if (proj) {
      projected.push_back(*proj);
    }
  }
  detail::draw_pose_axes(image, projected, pose_axis_thickness_);
}

void VisualizerNode::detections_callback(
  std::size_t cam_index, const perception_msgs::msg::DetectionArray::SharedPtr msg)
{
  if (cam_index < latest_detections_.size()) {
    latest_detections_[cam_index] = msg;
  }
}

void VisualizerNode::masks_callback(
  std::size_t cam_index, const perception_msgs::msg::SegmentationArray::SharedPtr msg)
{
  if (cam_index < latest_masks_.size()) {
    latest_masks_[cam_index] = msg;
  }
}

void VisualizerNode::camera_info_callback(
  std::size_t cam_index, const sensor_msgs::msg::CameraInfo::SharedPtr msg)
{
  if (cam_index >= latest_intrinsics_.size() || !msg) {
    return;
  }
  CameraIntrinsics intr;
  for (std::size_t i = 0; i < intr.k.size(); ++i) {
    intr.k[i] = msg->k[i];
  }
  intr.frame_id = msg->header.frame_id;
  latest_intrinsics_[cam_index] = std::move(intr);
}

void VisualizerNode::smoothed_poses_callback(
  const perception_msgs::msg::PoseWithMetaArray::SharedPtr msg)
{
  latest_poses_ = msg;
}

void VisualizerNode::pipeline_status_callback(
  const perception_msgs::msg::PipelineStatus::SharedPtr msg)
{
  pipeline_mode_ = msg->active_mode;
}

rcl_interfaces::msg::SetParametersResult VisualizerNode::on_set_parameters(
  const std::vector<rclcpp::Parameter> & params)
{
  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;
  for (const auto & p : params) {
    if (p.get_name() == "active_camera_index") {
      const int requested = static_cast<int>(p.as_int());
      if (requested < 0 ||
          static_cast<std::size_t>(requested) >= camera_namespaces_.size()) {
        result.successful = false;
        result.reason = "active_camera_index out of range";
        return result;
      }
      active_camera_index_ = static_cast<std::size_t>(requested);
      RCLCPP_INFO(
        get_logger(), "active_camera_index → %zu (ns='%s')",
        active_camera_index_, camera_namespaces_[active_camera_index_].c_str());
    } else if (p.get_name() == "enable_sam2_masks") {
      enable_sam2_masks_ = p.as_bool();
      RCLCPP_INFO(
        get_logger(), "enable_sam2_masks → %s",
        enable_sam2_masks_ ? "true" : "false");
    } else if (p.get_name() == "mask_alpha") {
      const double requested = p.as_double();
      if (requested < 0.0 || requested > 1.0) {
        result.successful = false;
        result.reason = "mask_alpha must be within [0.0, 1.0]";
        return result;
      }
      mask_alpha_ = static_cast<float>(requested);
      RCLCPP_INFO(get_logger(), "mask_alpha → %.2f", mask_alpha_);
    } else if (p.get_name() == "enable_pose_axes") {
      enable_pose_axes_ = p.as_bool();
      RCLCPP_INFO(
        get_logger(), "enable_pose_axes → %s",
        enable_pose_axes_ ? "true" : "false");
    } else if (p.get_name() == "axis_length_m") {
      const double requested = p.as_double();
      if (requested <= 0.0) {
        result.successful = false;
        result.reason = "axis_length_m must be > 0";
        return result;
      }
      axis_length_m_ = requested;
      RCLCPP_INFO(get_logger(), "axis_length_m → %.4f", axis_length_m_);
    } else if (p.get_name() == "pose_axis_thickness") {
      const int64_t requested = p.as_int();
      if (requested < 1 || requested > 10) {
        result.successful = false;
        result.reason = "pose_axis_thickness must be within [1, 10]";
        return result;
      }
      pose_axis_thickness_ = static_cast<int>(requested);
      RCLCPP_INFO(get_logger(), "pose_axis_thickness → %d", pose_axis_thickness_);
    }
  }
  return result;
}

}  // namespace perspective_grasp
