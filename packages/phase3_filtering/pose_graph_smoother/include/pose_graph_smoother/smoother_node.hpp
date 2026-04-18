#pragma once

#include <memory>
#include <string>
#include <vector>

#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <perception_msgs/msg/pose_with_meta_array.hpp>

namespace perspective_grasp
{

class SmootherNode : public rclcpp_lifecycle::LifecycleNode
{
public:
  explicit SmootherNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

  // Lifecycle callbacks
  using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;
  CallbackReturn on_configure(const rclcpp_lifecycle::State & state) override;
  CallbackReturn on_activate(const rclcpp_lifecycle::State & state) override;
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State & state) override;
  CallbackReturn on_cleanup(const rclcpp_lifecycle::State & state) override;

private:
  void filtered_poses_callback(const perception_msgs::msg::PoseWithMetaArray::SharedPtr msg);
  void broadcast_tf(const perception_msgs::msg::PoseWithMetaArray & msg);

  rclcpp::Subscription<perception_msgs::msg::PoseWithMetaArray>::SharedPtr sub_filtered_;
  rclcpp_lifecycle::LifecyclePublisher<perception_msgs::msg::PoseWithMetaArray>::SharedPtr pub_smoothed_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

  // Parameters
  int window_size_;
  double prior_noise_pos_;
  double prior_noise_rot_;
  std::string camera_frame_id_;
};

}  // namespace perspective_grasp
