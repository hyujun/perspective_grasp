#pragma once

#include <string>
#include <unordered_map>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <perception_msgs/msg/pipeline_status.hpp>
#include <perception_msgs/msg/associated_pose_array.hpp>
#include <perception_msgs/srv/set_mode.hpp>

#include "perception_meta_controller/detail/mode_logic.hpp"

namespace perspective_grasp
{

class MetaControllerNode : public rclcpp::Node
{
public:
  explicit MetaControllerNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  void handle_set_mode(
    const perception_msgs::srv::SetMode::Request::SharedPtr request,
    perception_msgs::srv::SetMode::Response::SharedPtr response);

  void associated_poses_callback(
    const perception_msgs::msg::AssociatedPoseArray::ConstSharedPtr & msg);

  void publish_status();

  rclcpp::Service<perception_msgs::srv::SetMode>::SharedPtr srv_set_mode_;
  rclcpp::Publisher<perception_msgs::msg::PipelineStatus>::SharedPtr pub_status_;
  rclcpp::Subscription<perception_msgs::msg::AssociatedPoseArray>::SharedPtr sub_associated_;
  rclcpp::TimerBase::SharedPtr status_timer_;

  std::string active_mode_;
  std::vector<std::string> active_nodes_;

  std::unordered_map<int, detail::ObjectVisibility> object_visibility_;
  int tracked_count_{0};
  int lost_count_{0};
  int num_cameras_{1};
};

}  // namespace perspective_grasp
