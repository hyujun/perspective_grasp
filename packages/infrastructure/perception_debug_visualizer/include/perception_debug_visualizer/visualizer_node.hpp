#pragma once

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.hpp>
#include <image_transport/image_transport.hpp>
#include <perception_msgs/msg/detection_array.hpp>
#include <perception_msgs/msg/pose_with_meta_array.hpp>
#include <perception_msgs/msg/pipeline_status.hpp>

#include <opencv2/opencv.hpp>

namespace perspective_grasp
{

class VisualizerNode : public rclcpp::Node
{
public:
  explicit VisualizerNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  void image_callback(const sensor_msgs::msg::Image::ConstSharedPtr & msg);
  void detections_callback(const perception_msgs::msg::DetectionArray::SharedPtr msg);
  void smoothed_poses_callback(const perception_msgs::msg::PoseWithMetaArray::SharedPtr msg);
  void pipeline_status_callback(const perception_msgs::msg::PipelineStatus::SharedPtr msg);

  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_image_;
  rclcpp::Subscription<perception_msgs::msg::DetectionArray>::SharedPtr sub_detections_;
  rclcpp::Subscription<perception_msgs::msg::PoseWithMetaArray>::SharedPtr sub_smoothed_;
  rclcpp::Subscription<perception_msgs::msg::PipelineStatus>::SharedPtr sub_status_;

  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_debug_image_;

  // Cached latest messages
  perception_msgs::msg::DetectionArray::SharedPtr latest_detections_;
  perception_msgs::msg::PoseWithMetaArray::SharedPtr latest_poses_;
  std::string pipeline_mode_;
};

}  // namespace perspective_grasp
