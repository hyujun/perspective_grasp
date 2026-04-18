#pragma once

#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include "pcl_merge_node/cloud_preprocessor.hpp"
#include "pcl_merge_node/overlap_filter.hpp"

namespace perspective_grasp {

class MergeNode : public rclcpp::Node {
public:
  explicit MergeNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

private:
  enum class Mode { PASSTHROUGH, MERGE_2, MERGE_N };

  // ---------- Parameters ----------
  std::vector<std::string> source_topics_;
  std::string target_frame_;
  double voxel_leaf_size_;
  double sync_slop_ms_;
  int sync_queue_size_;

  Mode mode_;

  // ---------- TF2 ----------
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  // ---------- Processing ----------
  CloudPreprocessor preprocessor_;
  OverlapFilter overlap_filter_;

  // ---------- Publishers ----------
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_merged_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr pub_diag_;

  // ---------- PASSTHROUGH mode ----------
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_single_;
  void passthroughCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);

  // ---------- MERGE_2 mode ----------
  using SyncPolicy2 = message_filters::sync_policies::ApproximateTime<
      sensor_msgs::msg::PointCloud2, sensor_msgs::msg::PointCloud2>;
  std::shared_ptr<message_filters::Subscriber<sensor_msgs::msg::PointCloud2>> sync_sub_a_;
  std::shared_ptr<message_filters::Subscriber<sensor_msgs::msg::PointCloud2>> sync_sub_b_;
  std::shared_ptr<message_filters::Synchronizer<SyncPolicy2>> sync2_;
  void merge2Callback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr& msg_a,
                      const sensor_msgs::msg::PointCloud2::ConstSharedPtr& msg_b);

  // ---------- MERGE_N mode ----------
  std::vector<rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr> subs_n_;
  std::vector<sensor_msgs::msg::PointCloud2::SharedPtr> latest_clouds_;
  std::mutex clouds_mutex_;
  rclcpp::TimerBase::SharedPtr merge_timer_;
  void mergeNTimerCallback();

  // ---------- Helpers ----------
  sensor_msgs::msg::PointCloud2 transformCloud(
      const sensor_msgs::msg::PointCloud2& cloud_in) const;
  void publishDiagnostics(const std::string& detail,
                          size_t input_total, size_t output_count,
                          double processing_ms);
};

}  // namespace perspective_grasp
