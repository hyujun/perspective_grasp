#include "perception_debug_visualizer/visualizer_node.hpp"

namespace perspective_grasp
{

VisualizerNode::VisualizerNode(const rclcpp::NodeOptions & options)
: rclcpp::Node("perception_debug_visualizer", options),
  pipeline_mode_("UNKNOWN")
{
  declare_parameter("image_topic", std::string("/camera/color/image_raw"));

  auto sensor_qos = rclcpp::QoS(1).best_effort();
  auto reliable_qos = rclcpp::QoS(1).reliable();

  std::string image_topic = get_parameter("image_topic").as_string();

  sub_image_ = create_subscription<sensor_msgs::msg::Image>(
    image_topic, sensor_qos,
    std::bind(&VisualizerNode::image_callback, this, std::placeholders::_1));

  sub_detections_ = create_subscription<perception_msgs::msg::DetectionArray>(
    "/yolo/detections", sensor_qos,
    std::bind(&VisualizerNode::detections_callback, this, std::placeholders::_1));

  sub_smoothed_ = create_subscription<perception_msgs::msg::PoseWithMetaArray>(
    "/smoother/smoothed_poses", sensor_qos,
    std::bind(&VisualizerNode::smoothed_poses_callback, this, std::placeholders::_1));

  sub_status_ = create_subscription<perception_msgs::msg::PipelineStatus>(
    "/meta_controller/active_pipeline", reliable_qos,
    std::bind(&VisualizerNode::pipeline_status_callback, this, std::placeholders::_1));

  pub_debug_image_ = create_publisher<sensor_msgs::msg::Image>(
    "/debug/image", rclcpp::QoS(1).best_effort());

  RCLCPP_INFO(get_logger(), "VisualizerNode started (stub - minimal overlay)");
}

void VisualizerNode::image_callback(const sensor_msgs::msg::Image::ConstSharedPtr & msg)
{
  // Minimal stub: just republish the image with a text overlay
  try {
    auto cv_ptr = cv_bridge::toCvCopy(msg, "bgr8");
    cv::putText(
      cv_ptr->image,
      "Mode: " + pipeline_mode_,
      cv::Point(10, 30),
      cv::FONT_HERSHEY_SIMPLEX, 0.8,
      cv::Scalar(0, 255, 0), 2);

    // TODO: overlay detection bounding boxes, pose axes, etc.

    pub_debug_image_->publish(*cv_ptr->toImageMsg());
  } catch (const cv_bridge::Exception & e) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "cv_bridge error: %s", e.what());
  }
}

void VisualizerNode::detections_callback(
  const perception_msgs::msg::DetectionArray::SharedPtr msg)
{
  latest_detections_ = msg;
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

}  // namespace perspective_grasp

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<perspective_grasp::VisualizerNode>());
  rclcpp::shutdown();
  return 0;
}
