#include "pcl_merge_node/merge_node.hpp"

#include <chrono>

#include <pcl/filters/voxel_grid.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.hpp>

namespace perspective_grasp {

MergeNode::MergeNode(const rclcpp::NodeOptions& options)
    : Node("pcl_merge_node", options),
      preprocessor_(PreprocessorParams{}),
      overlap_filter_(0.005) {
  // Declare parameters
  this->declare_parameter<std::vector<std::string>>(
      "source_topics",
      std::vector<std::string>{"/cam1/camera/depth/points"});
  this->declare_parameter<std::string>("target_frame", "ur5e_base_link");
  this->declare_parameter<double>("voxel_leaf_size", 0.002);
  this->declare_parameter<double>("table_height", 0.0);
  this->declare_parameter<double>("max_object_height", 0.30);
  this->declare_parameter<int>("outlier_mean_k", 30);
  this->declare_parameter<double>("outlier_stddev", 1.5);
  this->declare_parameter<double>("overlap_radius", 0.005);
  this->declare_parameter<double>("sync_slop_ms", 5.0);
  this->declare_parameter<int>("sync_queue_size", 10);

  // Read parameters
  source_topics_ = this->get_parameter("source_topics").as_string_array();
  target_frame_ = this->get_parameter("target_frame").as_string();
  voxel_leaf_size_ = this->get_parameter("voxel_leaf_size").as_double();
  sync_slop_ms_ = this->get_parameter("sync_slop_ms").as_double();
  sync_queue_size_ = this->get_parameter("sync_queue_size").as_int();

  PreprocessorParams pp;
  pp.table_height = this->get_parameter("table_height").as_double();
  pp.max_object_height = this->get_parameter("max_object_height").as_double();
  pp.outlier_mean_k = this->get_parameter("outlier_mean_k").as_int();
  pp.outlier_stddev = this->get_parameter("outlier_stddev").as_double();
  preprocessor_.setParams(pp);

  overlap_filter_.setOverlapRadius(this->get_parameter("overlap_radius").as_double());

  // TF2
  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  // QoS: BEST_EFFORT + depth 1 for sensor topics
  auto sensor_qos = rclcpp::SensorDataQoS().keep_last(1);

  // Publishers
  pub_merged_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
      "/merged/points", sensor_qos);
  pub_diag_ = this->create_publisher<diagnostic_msgs::msg::DiagnosticArray>(
      "/merged/diagnostics", 10);

  // Mode selection
  if (source_topics_.size() <= 1) {
    mode_ = Mode::PASSTHROUGH;
    std::string topic = source_topics_.empty()
                            ? "/cam1/camera/depth/points"
                            : source_topics_[0];
    RCLCPP_INFO(this->get_logger(), "PASSTHROUGH mode: subscribing to %s",
                topic.c_str());
    sub_single_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        topic, sensor_qos,
        std::bind(&MergeNode::passthroughCallback, this,
                  std::placeholders::_1));
  } else if (source_topics_.size() == 2) {
    mode_ = Mode::MERGE_2;
    RCLCPP_INFO(this->get_logger(),
                "MERGE_2 mode: syncing %s and %s",
                source_topics_[0].c_str(), source_topics_[1].c_str());

    rmw_qos_profile_t rmw_qos = rmw_qos_profile_sensor_data;
    rmw_qos.depth = 1;

    sync_sub_a_ =
        std::make_shared<message_filters::Subscriber<sensor_msgs::msg::PointCloud2>>(
            this, source_topics_[0], rmw_qos);
    sync_sub_b_ =
        std::make_shared<message_filters::Subscriber<sensor_msgs::msg::PointCloud2>>(
            this, source_topics_[1], rmw_qos);

    sync2_ = std::make_shared<message_filters::Synchronizer<SyncPolicy2>>(
        SyncPolicy2(static_cast<uint32_t>(sync_queue_size_)),
        *sync_sub_a_, *sync_sub_b_);
    sync2_->setMaxIntervalDuration(
        rclcpp::Duration::from_seconds(sync_slop_ms_ / 1000.0));
    sync2_->registerCallback(
        std::bind(&MergeNode::merge2Callback, this,
                  std::placeholders::_1, std::placeholders::_2));
  } else {
    mode_ = Mode::MERGE_N;
    RCLCPP_INFO(this->get_logger(), "MERGE_N mode: %zu sources",
                source_topics_.size());

    latest_clouds_.resize(source_topics_.size());
    for (size_t i = 0; i < source_topics_.size(); ++i) {
      subs_n_.push_back(
          this->create_subscription<sensor_msgs::msg::PointCloud2>(
              source_topics_[i], sensor_qos,
              [this, i](const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
                std::lock_guard<std::mutex> lock(clouds_mutex_);
                latest_clouds_[i] = msg;
              }));
    }

    // Merge at ~10 Hz
    merge_timer_ = this->create_wall_timer(
        std::chrono::milliseconds(100),
        std::bind(&MergeNode::mergeNTimerCallback, this));
  }
}

sensor_msgs::msg::PointCloud2
MergeNode::transformCloud(const sensor_msgs::msg::PointCloud2& cloud_in) const {
  sensor_msgs::msg::PointCloud2 cloud_out;
  try {
    auto transform = tf_buffer_->lookupTransform(
        target_frame_, cloud_in.header.frame_id,
        tf2::TimePointZero, tf2::durationFromSec(0.1));
    tf2::doTransform(cloud_in, cloud_out, transform);
  } catch (const tf2::TransformException& ex) {
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                         "TF transform failed: %s", ex.what());
    cloud_out = cloud_in;
  }
  cloud_out.header.frame_id = target_frame_;
  return cloud_out;
}

void MergeNode::passthroughCallback(
    const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
  auto t_start = std::chrono::steady_clock::now();

  auto transformed = transformCloud(*msg);

  // Convert, preprocess, convert back
  auto pcl_cloud = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
  pcl::fromROSMsg(transformed, *pcl_cloud);
  size_t input_count = pcl_cloud->size();

  auto filtered = preprocessor_.process(pcl_cloud);

  sensor_msgs::msg::PointCloud2 out_msg;
  pcl::toROSMsg(*filtered, out_msg);
  out_msg.header.frame_id = target_frame_;
  out_msg.header.stamp = msg->header.stamp;
  pub_merged_->publish(out_msg);

  auto t_end = std::chrono::steady_clock::now();
  double ms = std::chrono::duration<double, std::milli>(t_end - t_start).count();
  publishDiagnostics("PASSTHROUGH", input_count, filtered->size(), ms);
}

void MergeNode::merge2Callback(
    const sensor_msgs::msg::PointCloud2::ConstSharedPtr& msg_a,
    const sensor_msgs::msg::PointCloud2::ConstSharedPtr& msg_b) {
  auto t_start = std::chrono::steady_clock::now();

  // 1. Transform each to target_frame
  auto tf_a = transformCloud(*msg_a);
  auto tf_b = transformCloud(*msg_b);

  // 2. Convert to PCL
  auto pcl_a = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
  auto pcl_b = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
  pcl::fromROSMsg(tf_a, *pcl_a);
  pcl::fromROSMsg(tf_b, *pcl_b);
  size_t input_total = pcl_a->size() + pcl_b->size();

  // 3. Preprocess each
  auto filt_a = preprocessor_.process(pcl_a);
  auto filt_b = preprocessor_.process(pcl_b);

  // 4. Concatenate
  auto merged = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
  *merged = *filt_a + *filt_b;

  // 5. VoxelGrid dedup
  auto deduped = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
  pcl::VoxelGrid<pcl::PointXYZ> voxel;
  voxel.setInputCloud(merged);
  float leaf = static_cast<float>(voxel_leaf_size_);
  voxel.setLeafSize(leaf, leaf, leaf);
  voxel.filter(*deduped);

  // 6. Overlap scoring
  overlap_filter_.process(deduped, filt_a, filt_b);

  // 7. Publish
  sensor_msgs::msg::PointCloud2 out_msg;
  pcl::toROSMsg(*deduped, out_msg);
  out_msg.header.frame_id = target_frame_;
  out_msg.header.stamp = msg_a->header.stamp;
  pub_merged_->publish(out_msg);

  auto t_end = std::chrono::steady_clock::now();
  double ms = std::chrono::duration<double, std::milli>(t_end - t_start).count();
  publishDiagnostics("MERGE_2", input_total, deduped->size(), ms);
}

void MergeNode::mergeNTimerCallback() {
  auto t_start = std::chrono::steady_clock::now();

  std::vector<sensor_msgs::msg::PointCloud2::SharedPtr> snapshots;
  {
    std::lock_guard<std::mutex> lock(clouds_mutex_);
    snapshots = latest_clouds_;
  }

  // Collect valid clouds
  std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> filtered_clouds;
  size_t input_total = 0;
  rclcpp::Time latest_stamp = this->now();

  for (const auto& cloud_msg : snapshots) {
    if (!cloud_msg) {
      continue;
    }
    auto tf_cloud = transformCloud(*cloud_msg);
    auto pcl_cloud = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
    pcl::fromROSMsg(tf_cloud, *pcl_cloud);
    input_total += pcl_cloud->size();
    auto filtered = preprocessor_.process(pcl_cloud);
    filtered_clouds.push_back(filtered);
    latest_stamp = cloud_msg->header.stamp;
  }

  if (filtered_clouds.empty()) {
    return;
  }

  // Concatenate all
  auto merged = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
  for (const auto& cloud : filtered_clouds) {
    *merged += *cloud;
  }

  // VoxelGrid dedup
  auto deduped = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
  pcl::VoxelGrid<pcl::PointXYZ> voxel;
  voxel.setInputCloud(merged);
  float leaf = static_cast<float>(voxel_leaf_size_);
  voxel.setLeafSize(leaf, leaf, leaf);
  voxel.filter(*deduped);

  // Overlap scoring (use first two clouds if available)
  if (filtered_clouds.size() >= 2) {
    overlap_filter_.process(deduped, filtered_clouds[0], filtered_clouds[1]);
  }

  // Publish
  sensor_msgs::msg::PointCloud2 out_msg;
  pcl::toROSMsg(*deduped, out_msg);
  out_msg.header.frame_id = target_frame_;
  out_msg.header.stamp = latest_stamp;
  pub_merged_->publish(out_msg);

  auto t_end = std::chrono::steady_clock::now();
  double ms = std::chrono::duration<double, std::milli>(t_end - t_start).count();
  publishDiagnostics("MERGE_N", input_total, deduped->size(), ms);
}

void MergeNode::publishDiagnostics(const std::string& detail,
                                   size_t input_total,
                                   size_t output_count,
                                   double processing_ms) {
  diagnostic_msgs::msg::DiagnosticArray diag_msg;
  diag_msg.header.stamp = this->now();

  diagnostic_msgs::msg::DiagnosticStatus status;
  status.level = diagnostic_msgs::msg::DiagnosticStatus::OK;
  status.name = "pcl_merge_node";
  status.message = detail;

  diagnostic_msgs::msg::KeyValue kv_input;
  kv_input.key = "input_points";
  kv_input.value = std::to_string(input_total);
  status.values.push_back(kv_input);

  diagnostic_msgs::msg::KeyValue kv_output;
  kv_output.key = "output_points";
  kv_output.value = std::to_string(output_count);
  status.values.push_back(kv_output);

  diagnostic_msgs::msg::KeyValue kv_time;
  kv_time.key = "processing_ms";
  kv_time.value = std::to_string(processing_ms);
  status.values.push_back(kv_time);

  diag_msg.status.push_back(status);
  pub_diag_->publish(diag_msg);
}

}  // namespace perspective_grasp
