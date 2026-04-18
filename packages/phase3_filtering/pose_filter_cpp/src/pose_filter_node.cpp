#include "pose_filter_cpp/pose_filter_node.hpp"

#include <tf2_eigen/tf2_eigen.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <diagnostic_msgs/msg/diagnostic_status.hpp>
#include <diagnostic_msgs/msg/key_value.hpp>

#include <algorithm>
#include <cmath>

namespace perspective_grasp {

PoseFilterNode::PoseFilterNode(const rclcpp::NodeOptions& options)
    : Node("pose_filter", options) {
  // Process-noise parameters (continuous-time σ, units: ·/√s)
  this->declare_parameter("process_noise_pos", 0.001);
  this->declare_parameter("process_noise_rot", 0.01);
  this->declare_parameter("process_noise_omega", 0.1);
  this->declare_parameter("process_noise_vel", 0.1);

  // Measurement-noise parameters (units: m and rad)
  this->declare_parameter("meas_noise_pos", 0.005);
  this->declare_parameter("meas_noise_rot", 0.05);

  this->declare_parameter("iekf_max_iterations", 3);
  this->declare_parameter("mahalanobis_threshold", 16.81);
  this->declare_parameter("publish_rate_hz", 30.0);
  this->declare_parameter("stale_timeout_sec", 2.0);
  this->declare_parameter("stale_predict_threshold_sec", 1.0);
  this->declare_parameter("output_frame_id", "camera_color_optical_frame");
  this->declare_parameter("use_associated_input", true);

  // Fitness → noise-scale mapping
  this->declare_parameter("fitness_reference", 0.001);
  this->declare_parameter("noise_scale_min", 0.1);
  this->declare_parameter("noise_scale_max", 10.0);

  // Source weights (lower = more trusted) — used in legacy mode
  this->declare_parameter("source_weights.yolo_tracker", 1.0);
  this->declare_parameter("source_weights.foundationpose", 0.3);
  this->declare_parameter("source_weights.megapose", 0.5);
  this->declare_parameter("source_weights.bundlesdf", 0.8);

  filter_config_.process_noise_pos =
      this->get_parameter("process_noise_pos").as_double();
  filter_config_.process_noise_rot =
      this->get_parameter("process_noise_rot").as_double();
  filter_config_.process_noise_omega =
      this->get_parameter("process_noise_omega").as_double();
  filter_config_.process_noise_vel =
      this->get_parameter("process_noise_vel").as_double();
  filter_config_.meas_noise_pos =
      this->get_parameter("meas_noise_pos").as_double();
  filter_config_.meas_noise_rot =
      this->get_parameter("meas_noise_rot").as_double();
  filter_config_.max_iterations =
      this->get_parameter("iekf_max_iterations").as_int();
  filter_config_.mahalanobis_threshold =
      this->get_parameter("mahalanobis_threshold").as_double();
  stale_timeout_sec_ = this->get_parameter("stale_timeout_sec").as_double();
  stale_predict_threshold_sec_ =
      this->get_parameter("stale_predict_threshold_sec").as_double();
  output_frame_id_ = this->get_parameter("output_frame_id").as_string();
  use_associated_input_ = this->get_parameter("use_associated_input").as_bool();
  fitness_reference_ = this->get_parameter("fitness_reference").as_double();
  noise_scale_min_ = this->get_parameter("noise_scale_min").as_double();
  noise_scale_max_ = this->get_parameter("noise_scale_max").as_double();

  source_weights_["yolo_tracker"] =
      this->get_parameter("source_weights.yolo_tracker").as_double();
  source_weights_["foundationpose"] =
      this->get_parameter("source_weights.foundationpose").as_double();
  source_weights_["megapose"] =
      this->get_parameter("source_weights.megapose").as_double();
  source_weights_["bundlesdf"] =
      this->get_parameter("source_weights.bundlesdf").as_double();

  auto sensor_qos = rclcpp::SensorDataQoS();

  if (use_associated_input_) {
    // Multi-camera mode: single subscription to cross_camera_associator output
    associated_sub_ =
        this->create_subscription<perception_msgs::msg::AssociatedPoseArray>(
            "/associated/poses", sensor_qos,
            std::bind(&PoseFilterNode::associatedPosesCallback, this,
                      std::placeholders::_1));
    RCLCPP_INFO(this->get_logger(),
                "PoseFilterNode: associated mode (subscribing /associated/poses)");
  } else {
    // Legacy single-camera mode: subscribe to multiple raw_poses topics
    auto make_cb = [this](const std::string& source, double weight) {
      return [this, source, weight](
                 const perception_msgs::msg::PoseWithMetaArray::ConstSharedPtr&
                     msg) { this->rawPosesCallback(msg, source, weight); };
    };

    yolo_sub_ =
        this->create_subscription<perception_msgs::msg::PoseWithMetaArray>(
            "/yolo_tracker/raw_poses", sensor_qos,
            make_cb("yolo_tracker", source_weights_["yolo_tracker"]));
    fp_sub_ =
        this->create_subscription<perception_msgs::msg::PoseWithMetaArray>(
            "/foundationpose/raw_poses", sensor_qos,
            make_cb("foundationpose", source_weights_["foundationpose"]));
    megapose_sub_ =
        this->create_subscription<perception_msgs::msg::PoseWithMetaArray>(
            "/megapose/raw_poses", sensor_qos,
            make_cb("megapose", source_weights_["megapose"]));
    bundlesdf_sub_ =
        this->create_subscription<perception_msgs::msg::PoseWithMetaArray>(
            "/bundlesdf/raw_poses", sensor_qos,
            make_cb("bundlesdf", source_weights_["bundlesdf"]));
    RCLCPP_INFO(this->get_logger(),
                "PoseFilterNode: legacy mode (4 raw_poses subscriptions)");
  }

  // Publishers
  filtered_pub_ = this->create_publisher<perception_msgs::msg::PoseWithMetaArray>(
      "/pose_filter/filtered_poses", rclcpp::QoS(1).best_effort());
  cov_pub_ = this->create_publisher<perception_msgs::msg::PoseCovarianceArray>(
      "/pose_filter/covariance", rclcpp::QoS(1).best_effort());
  diag_pub_ = this->create_publisher<diagnostic_msgs::msg::DiagnosticArray>(
      "/pose_filter/diagnostics", rclcpp::QoS(1).best_effort());

  // TF broadcaster
  tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

  // Fixed-rate publish timer
  double rate = this->get_parameter("publish_rate_hz").as_double();
  pub_timer_ = this->create_wall_timer(
      std::chrono::duration<double>(1.0 / rate),
      std::bind(&PoseFilterNode::publishTimerCallback, this));

  RCLCPP_INFO(this->get_logger(), "PoseFilterNode initialized (%.0f Hz, frame: %s)",
              rate, output_frame_id_.c_str());
}

void PoseFilterNode::associatedPosesCallback(
    const perception_msgs::msg::AssociatedPoseArray::ConstSharedPtr& msg) {
  for (const auto& ap : msg->poses) {
    int id = ap.global_id;

    Eigen::Isometry3d meas;
    tf2::fromMsg(ap.pose.pose, meas);

    auto it = filters_.find(id);
    if (it == filters_.end()) {
      FilterState fs;
      fs.filter = IekfSe3(filter_config_);
      fs.class_name = ap.class_name;
      fs.last_update = this->now();
      filters_[id] = std::move(fs);
      it = filters_.find(id);
    }

    auto& fs = it->second;
    fs.class_name = ap.class_name;  // follow associator re-classification

    const double dt = (this->now() - fs.last_update).seconds();
    if (dt > stale_predict_threshold_sec_) {
      RCLCPP_WARN(this->get_logger(),
                  "Filter %d went stale (dt=%.2fs); reinitializing from "
                  "current measurement",
                  id, dt);
      fs.filter.reset(meas);
    } else if (dt > 0.0) {
      fs.filter.predict(dt);
    }

    // Compute noise scale: more cameras → tighter; higher fitness → looser.
    double noise_scale = 1.0;
    if (ap.num_observing_cameras > 1) {
      noise_scale =
          1.0 / std::sqrt(static_cast<double>(ap.num_observing_cameras));
    }
    if (!ap.source_fitness_scores.empty() && fitness_reference_ > 0.0) {
      const float best_fitness = *std::min_element(
          ap.source_fitness_scores.begin(), ap.source_fitness_scores.end());
      if (best_fitness > 0.0f) {
        noise_scale *= static_cast<double>(best_fitness) / fitness_reference_;
      }
    }
    noise_scale = std::clamp(noise_scale, noise_scale_min_, noise_scale_max_);

    const bool accepted = fs.filter.update(meas, noise_scale);
    ++total_updates_;
    if (!accepted) ++total_rejections_;
    fs.last_update = this->now();
  }
}

void PoseFilterNode::rawPosesCallback(
    const perception_msgs::msg::PoseWithMetaArray::ConstSharedPtr& msg,
    const std::string& /*source_name*/, double noise_scale) {
  for (const auto& pose_meta : msg->poses) {
    int id = pose_meta.object_id;

    Eigen::Isometry3d meas;
    tf2::fromMsg(pose_meta.pose.pose, meas);

    auto it = filters_.find(id);
    if (it == filters_.end()) {
      FilterState fs;
      fs.filter = IekfSe3(filter_config_);
      fs.class_name = pose_meta.class_name;
      fs.last_update = this->now();
      filters_[id] = std::move(fs);
      it = filters_.find(id);
    }

    auto& fs = it->second;
    fs.class_name = pose_meta.class_name;

    const double dt = (this->now() - fs.last_update).seconds();
    if (dt > stale_predict_threshold_sec_) {
      RCLCPP_WARN(this->get_logger(),
                  "Filter %d went stale (dt=%.2fs); reinitializing from "
                  "current measurement",
                  id, dt);
      fs.filter.reset(meas);
    } else if (dt > 0.0) {
      fs.filter.predict(dt);
    }

    const bool accepted = fs.filter.update(meas, noise_scale);
    ++total_updates_;
    if (!accepted) ++total_rejections_;
    fs.last_update = this->now();
  }
}

void PoseFilterNode::publishTimerCallback() {
  pruneStaleFilters();

  auto now = this->now();

  perception_msgs::msg::PoseWithMetaArray filtered_msg;
  filtered_msg.header.stamp = now;
  filtered_msg.header.frame_id = output_frame_id_;

  perception_msgs::msg::PoseCovarianceArray cov_msg;
  cov_msg.header = filtered_msg.header;

  for (auto& [id, fs] : filters_) {
    if (!fs.filter.initialized()) continue;

    auto pose = fs.filter.pose();

    // Filtered pose message
    perception_msgs::msg::PoseWithMeta pm;
    pm.object_id = id;
    pm.class_name = fs.class_name;
    pm.source = "iekf";
    pm.confidence = 1.0f;
    pm.fitness_score = 0.0f;
    pm.pose.header.stamp = now;
    pm.pose.header.frame_id = output_frame_id_;
    pm.pose.pose = tf2::toMsg(pose);
    filtered_msg.poses.push_back(pm);

    // Covariance message
    perception_msgs::msg::PoseCovarianceStamped pc;
    pc.object_id = id;
    pc.pose = pm.pose;
    auto cov6 = fs.filter.poseCovariance();
    for (int r = 0; r < 6; ++r)
      for (int c = 0; c < 6; ++c)
        pc.covariance[static_cast<size_t>(r * 6 + c)] = cov6(r, c);
    cov_msg.poses.push_back(pc);

    // TF broadcast
    geometry_msgs::msg::TransformStamped tf;
    tf.header.stamp = now;
    tf.header.frame_id = output_frame_id_;
    tf.child_frame_id = "object_" + std::to_string(id) + "_filtered";
    tf.transform.translation.x = pose.translation().x();
    tf.transform.translation.y = pose.translation().y();
    tf.transform.translation.z = pose.translation().z();
    Eigen::Quaterniond q(pose.linear());
    tf.transform.rotation.x = q.x();
    tf.transform.rotation.y = q.y();
    tf.transform.rotation.z = q.z();
    tf.transform.rotation.w = q.w();
    tf_broadcaster_->sendTransform(tf);
  }

  if (!filtered_msg.poses.empty()) {
    filtered_pub_->publish(filtered_msg);
    cov_pub_->publish(cov_msg);
  }

  // Diagnostics — always publish so subscribers see heartbeat even when idle.
  diagnostic_msgs::msg::DiagnosticArray diag_msg;
  diag_msg.header.stamp = now;
  diagnostic_msgs::msg::DiagnosticStatus status;
  status.name = "pose_filter";
  status.hardware_id = "";
  const size_t active_filters = filters_.size();
  const size_t published = filtered_msg.poses.size();
  const uint64_t rejections = total_rejections_;
  const uint64_t updates = total_updates_;
  status.level = active_filters == 0
                     ? diagnostic_msgs::msg::DiagnosticStatus::WARN
                     : diagnostic_msgs::msg::DiagnosticStatus::OK;
  status.message = active_filters == 0 ? "no active filters"
                                       : "tracking " +
                                             std::to_string(active_filters) +
                                             " object(s)";
  auto kv = [](const std::string& key, const std::string& value) {
    diagnostic_msgs::msg::KeyValue pair;
    pair.key = key;
    pair.value = value;
    return pair;
  };
  status.values.push_back(kv("active_filters", std::to_string(active_filters)));
  status.values.push_back(kv("published_filters", std::to_string(published)));
  status.values.push_back(kv("total_updates", std::to_string(updates)));
  status.values.push_back(kv("total_rejections", std::to_string(rejections)));
  diag_msg.status.push_back(status);
  diag_pub_->publish(diag_msg);
}

void PoseFilterNode::pruneStaleFilters() {
  auto now = this->now();
  for (auto it = filters_.begin(); it != filters_.end();) {
    double age = (now - it->second.last_update).seconds();
    if (age > stale_timeout_sec_) {
      RCLCPP_INFO(this->get_logger(), "Pruning stale filter for object_%d", it->first);
      it = filters_.erase(it);
    } else {
      ++it;
    }
  }
}

}  // namespace perspective_grasp
