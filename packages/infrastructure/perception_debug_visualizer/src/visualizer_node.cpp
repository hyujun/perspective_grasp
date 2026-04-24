#include "perception_debug_visualizer/visualizer_node.hpp"

#include "perception_debug_visualizer/detail/overlay.hpp"

#include <rcl_interfaces/msg/set_parameters_result.hpp>

#include <algorithm>
#include <cstddef>

namespace perspective_grasp
{

namespace {
constexpr const char * kDefaultImageSuffix = "camera/color/image_raw";
constexpr const char * kDefaultDetectionSuffix = "yolo/detections";
constexpr const char * kDefaultMasksSuffix = "sam2/masks";
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
  declare_parameter<bool>("enable_sam2_masks", true);
  declare_parameter<double>("mask_alpha", 0.4);
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
  const std::string legacy_image_topic = get_parameter("image_topic").as_string();
  enable_sam2_masks_ = get_parameter("enable_sam2_masks").as_bool();
  mask_alpha_ = static_cast<float>(
    std::clamp(get_parameter("mask_alpha").as_double(), 0.0, 1.0));
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
  latest_detections_.assign(camera_namespaces_.size(), nullptr);
  latest_masks_.assign(camera_namespaces_.size(), nullptr);

  for (std::size_t i = 0; i < camera_namespaces_.size(); ++i) {
    const std::string & ns = camera_namespaces_[i];
    std::string image_topic = compose_topic(ns, image_suffix);
    const std::string det_topic = compose_topic(ns, det_suffix);
    const std::string masks_topic = compose_topic(ns, masks_suffix);

    if (i == 0 && !legacy_image_topic.empty()) {
      RCLCPP_INFO(
        get_logger(),
        "'image_topic' parameter overrides camera 0 image topic to '%s' (legacy path)",
        legacy_image_topic.c_str());
      image_topic = legacy_image_topic;
    }

    RCLCPP_INFO(
      get_logger(),
      "cam[%zu] ns='%s' image='%s' detections='%s' masks='%s'",
      i, ns.c_str(), image_topic.c_str(), det_topic.c_str(),
      masks_topic.c_str());

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
    detail::draw_mode_overlay(cv_ptr->image, mode_label);
    if (enable_sam2_masks_ && cam_index < latest_masks_.size() &&
        latest_masks_[cam_index]) {
      detail::draw_masks(cv_ptr->image, *latest_masks_[cam_index], mask_alpha_);
    }
    if (cam_index < latest_detections_.size() && latest_detections_[cam_index]) {
      detail::draw_detections(cv_ptr->image, *latest_detections_[cam_index]);
    }
    pub_debug_image_->publish(*cv_ptr->toImageMsg());
  } catch (const cv_bridge::Exception & e) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "cv_bridge error: %s", e.what());
  }
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
    }
  }
  return result;
}

}  // namespace perspective_grasp
