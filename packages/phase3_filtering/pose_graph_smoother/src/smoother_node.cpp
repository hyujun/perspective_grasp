#include "pose_graph_smoother/smoother_node.hpp"

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <rclcpp/qos.hpp>

namespace perspective_grasp
{

SmootherNode::SmootherNode(const rclcpp::NodeOptions & options)
: rclcpp_lifecycle::LifecycleNode("pose_graph_smoother", options)
{
#ifdef HAS_GTSAM
  declare_parameter("window_size", 20);
  declare_parameter("prior_noise_pos", 0.01);
  declare_parameter("prior_noise_rot", 0.05);
#endif
  // Fallback parent frame for TF if the incoming message has no header.frame_id.
  // Normally the smoother preserves the upstream frame verbatim.
  declare_parameter("fallback_parent_frame",
                    std::string("camera_color_optical_frame"));
}

SmootherNode::CallbackReturn SmootherNode::on_configure(
  const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_INFO(get_logger(), "Configuring SmootherNode");

#ifdef HAS_GTSAM
  window_size_ = static_cast<int>(get_parameter("window_size").as_int());
  prior_noise_pos_ = get_parameter("prior_noise_pos").as_double();
  prior_noise_rot_ = get_parameter("prior_noise_rot").as_double();
#endif
  fallback_parent_frame_ =
    get_parameter("fallback_parent_frame").as_string();

  auto sensor_qos = rclcpp::QoS(1).best_effort();
  sub_filtered_ = create_subscription<perception_msgs::msg::PoseWithMetaArray>(
    "/pose_filter/filtered_poses", sensor_qos,
    std::bind(&SmootherNode::filtered_poses_callback, this, std::placeholders::_1));

  pub_smoothed_ = create_publisher<perception_msgs::msg::PoseWithMetaArray>(
    "/smoother/smoothed_poses", rclcpp::QoS(1).best_effort());

  tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

#ifdef HAS_GTSAM
  RCLCPP_WARN(
    get_logger(),
    "GTSAM is linked but the sliding-window optimizer is not implemented yet; "
    "falling back to passthrough so /smoother/smoothed_poses keeps flowing.");
#else
  RCLCPP_INFO(get_logger(),
    "SmootherNode configured (passthrough mode - GTSAM not available)");
#endif
  return CallbackReturn::SUCCESS;
}

SmootherNode::CallbackReturn SmootherNode::on_activate(
  const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_INFO(get_logger(), "Activating SmootherNode");
  pub_smoothed_->on_activate();
  return CallbackReturn::SUCCESS;
}

SmootherNode::CallbackReturn SmootherNode::on_deactivate(
  const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_INFO(get_logger(), "Deactivating SmootherNode");
  pub_smoothed_->on_deactivate();
  return CallbackReturn::SUCCESS;
}

SmootherNode::CallbackReturn SmootherNode::on_cleanup(
  const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_INFO(get_logger(), "Cleaning up SmootherNode");
  sub_filtered_.reset();
  pub_smoothed_.reset();
  tf_broadcaster_.reset();
  return CallbackReturn::SUCCESS;
}

void SmootherNode::filtered_poses_callback(
  const perception_msgs::msg::PoseWithMetaArray::SharedPtr msg)
{
  // Until the GTSAM sliding window is implemented, both build paths republish
  // the input unchanged so downstream consumers don't go silent.
  if (pub_smoothed_->is_activated()) {
    pub_smoothed_->publish(*msg);
    broadcast_tf(*msg);
  }
}

void SmootherNode::broadcast_tf(const perception_msgs::msg::PoseWithMetaArray & msg)
{
  // Preserve the upstream parent frame; fall back to the configured default
  // only when the incoming header has none.
  const std::string parent_frame =
    msg.header.frame_id.empty() ? fallback_parent_frame_ : msg.header.frame_id;

  for (const auto & pwm : msg.poses) {
    geometry_msgs::msg::TransformStamped t;
    t.header = msg.header;
    t.header.frame_id = parent_frame;
    t.child_frame_id = "object_" + std::to_string(pwm.object_id) + "_smoothed";
    t.transform.translation.x = pwm.pose.pose.position.x;
    t.transform.translation.y = pwm.pose.pose.position.y;
    t.transform.translation.z = pwm.pose.pose.position.z;
    t.transform.rotation = pwm.pose.pose.orientation;
    tf_broadcaster_->sendTransform(t);
  }
}

}  // namespace perspective_grasp
