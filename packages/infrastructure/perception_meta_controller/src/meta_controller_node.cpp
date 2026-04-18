#include "perception_meta_controller/meta_controller_node.hpp"

namespace perspective_grasp
{

MetaControllerNode::MetaControllerNode(const rclcpp::NodeOptions & options)
: rclcpp::Node("perception_meta_controller", options),
  active_mode_("NORMAL")
{
  declare_parameter("status_publish_rate_hz", 1.0);
  declare_parameter("default_mode", std::string("NORMAL"));
  declare_parameter("num_cameras", 1);
  declare_parameter("stale_object_timeout_sec", 3.0);

  active_mode_ = get_parameter("default_mode").as_string();
  num_cameras_ = static_cast<int>(get_parameter("num_cameras").as_int());
  active_nodes_ = detail::compute_active_nodes(active_mode_, num_cameras_);

  srv_set_mode_ = create_service<perception_msgs::srv::SetMode>(
    "/meta_controller/set_mode",
    std::bind(
      &MetaControllerNode::handle_set_mode, this,
      std::placeholders::_1, std::placeholders::_2));

  pub_status_ = create_publisher<perception_msgs::msg::PipelineStatus>(
    "/meta_controller/active_pipeline", rclcpp::QoS(1).reliable());

  sub_associated_ = create_subscription<perception_msgs::msg::AssociatedPoseArray>(
    "/associated/poses", rclcpp::SensorDataQoS(),
    std::bind(&MetaControllerNode::associated_poses_callback, this,
              std::placeholders::_1));

  double rate = get_parameter("status_publish_rate_hz").as_double();
  auto period = std::chrono::duration<double>(1.0 / rate);
  status_timer_ = create_wall_timer(
    std::chrono::duration_cast<std::chrono::nanoseconds>(period),
    std::bind(&MetaControllerNode::publish_status, this));

  RCLCPP_INFO(get_logger(), "MetaControllerNode started (mode: %s, cameras: %d)",
              active_mode_.c_str(), num_cameras_);
}

void MetaControllerNode::handle_set_mode(
  const perception_msgs::srv::SetMode::Request::SharedPtr request,
  perception_msgs::srv::SetMode::Response::SharedPtr response)
{
  RCLCPP_INFO(get_logger(), "SetMode request: %s", request->mode.c_str());

  if (detail::is_valid_mode(request->mode)) {
    active_mode_ = request->mode;
    active_nodes_ = detail::compute_active_nodes(active_mode_, num_cameras_);
    response->success = true;
    response->active_pipeline = active_mode_;
  } else {
    response->success = false;
    response->active_pipeline = active_mode_;
    RCLCPP_WARN(get_logger(), "Unknown mode: %s", request->mode.c_str());
  }
}

void MetaControllerNode::associated_poses_callback(
  const perception_msgs::msg::AssociatedPoseArray::ConstSharedPtr & msg)
{
  const double timeout = get_parameter("stale_object_timeout_sec").as_double();
  const auto counts = detail::update_visibility(
    object_visibility_, *msg, now(), timeout);
  tracked_count_ = counts.tracked;
  lost_count_ = counts.lost;
}

void MetaControllerNode::publish_status()
{
  perception_msgs::msg::PipelineStatus status;
  status.header.stamp = now();
  status.active_mode = active_mode_;
  status.active_nodes = active_nodes_;
  status.gpu_memory_usage_percent = 0.0F;
  status.tracked_object_count = tracked_count_;
  status.lost_object_count = lost_count_;
  pub_status_->publish(status);
}

}  // namespace perspective_grasp
