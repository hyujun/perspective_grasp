#include "pose_graph_smoother/smoother_node.hpp"

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <rclcpp/qos.hpp>

namespace perspective_grasp
{

SmootherNode::SmootherNode(const rclcpp::NodeOptions & options)
: rclcpp_lifecycle::LifecycleNode("pose_graph_smoother", options)
{
  declare_parameter("window_size", 20);
  declare_parameter("prior_noise_pos", 0.01);
  declare_parameter("prior_noise_rot", 0.05);
  declare_parameter("camera_frame_id", std::string("camera_color_optical_frame"));
}

SmootherNode::CallbackReturn SmootherNode::on_configure(
  const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_INFO(get_logger(), "Configuring SmootherNode");

  window_size_ = static_cast<int>(get_parameter("window_size").as_int());
  prior_noise_pos_ = get_parameter("prior_noise_pos").as_double();
  prior_noise_rot_ = get_parameter("prior_noise_rot").as_double();
  camera_frame_id_ = get_parameter("camera_frame_id").as_string();

  (void)window_size_;
  (void)prior_noise_pos_;
  (void)prior_noise_rot_;

  auto sensor_qos = rclcpp::QoS(1).best_effort();
  sub_filtered_ = create_subscription<perception_msgs::msg::PoseWithMetaArray>(
    "/pose_filter/filtered_poses", sensor_qos,
    std::bind(&SmootherNode::filtered_poses_callback, this, std::placeholders::_1));

  pub_smoothed_ = create_publisher<perception_msgs::msg::PoseWithMetaArray>(
    "/smoother/smoothed_poses", rclcpp::QoS(1).best_effort());

  tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

  RCLCPP_INFO(get_logger(), "SmootherNode configured (passthrough mode - GTSAM not available)");
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
#ifdef HAS_GTSAM
  // TODO: Add poses to GTSAM sliding window, optimize, publish result
  (void)msg;
#else
  // Passthrough mode: republish filtered as smoothed
  if (pub_smoothed_->is_activated()) {
    pub_smoothed_->publish(*msg);
    broadcast_tf(*msg);
  }
#endif
}

void SmootherNode::broadcast_tf(const perception_msgs::msg::PoseWithMetaArray & msg)
{
  for (const auto & pwm : msg.poses) {
    geometry_msgs::msg::TransformStamped t;
    t.header = msg.header;
    t.header.frame_id = camera_frame_id_;
    t.child_frame_id = "object_" + std::to_string(pwm.object_id);
    t.transform.translation.x = pwm.pose.pose.position.x;
    t.transform.translation.y = pwm.pose.pose.position.y;
    t.transform.translation.z = pwm.pose.pose.position.z;
    t.transform.rotation = pwm.pose.pose.orientation;
    tf_broadcaster_->sendTransform(t);
  }
}

}  // namespace perspective_grasp

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<perspective_grasp::SmootherNode>();
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node->get_node_base_interface());
  executor.spin();
  rclcpp::shutdown();
  return 0;
}
