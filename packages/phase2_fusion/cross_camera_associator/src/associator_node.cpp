#include "cross_camera_associator/associator_node.hpp"

#include <cmath>
#include <limits>
#include <map>
#include <set>
#include <string>
#include <vector>

#include <Eigen/Dense>
#include <tf2_eigen/tf2_eigen.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <diagnostic_msgs/msg/diagnostic_status.hpp>
#include <diagnostic_msgs/msg/key_value.hpp>

namespace perspective_grasp {

AssociatorNode::AssociatorNode(const rclcpp::NodeOptions& options)
    : Node("cross_camera_associator", options) {
  // Declare parameters
  this->declare_parameter<std::vector<std::string>>("camera_namespaces",
                                                     {"/cam0", "/cam1"});
  this->declare_parameter<std::string>("base_frame", "ur5e_base_link");
  this->declare_parameter<double>("association_distance_threshold", 0.05);
  this->declare_parameter<double>("lambda_rot", 0.1);
  this->declare_parameter<double>("temporal_threshold", 0.10);
  this->declare_parameter<double>("max_snapshot_age_ms", 50.0);
  this->declare_parameter<double>("association_rate_hz", 30.0);
  this->declare_parameter<double>("stale_object_timeout_sec", 3.0);

  // Read parameters
  camera_namespaces_ =
      this->get_parameter("camera_namespaces").as_string_array();
  base_frame_ = this->get_parameter("base_frame").as_string();
  association_distance_threshold_ =
      this->get_parameter("association_distance_threshold").as_double();
  lambda_rot_ = this->get_parameter("lambda_rot").as_double();
  temporal_threshold_ = this->get_parameter("temporal_threshold").as_double();
  max_snapshot_age_ms_ = this->get_parameter("max_snapshot_age_ms").as_double();
  association_rate_hz_ = this->get_parameter("association_rate_hz").as_double();
  stale_object_timeout_sec_ =
      this->get_parameter("stale_object_timeout_sec").as_double();

  id_manager_ = GlobalIdManager(association_distance_threshold_);

  // TF2
  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  // QoS: BEST_EFFORT + depth 1 for control-path topics
  auto qos = rclcpp::QoS(1).best_effort();

  // Create subscriptions for each camera namespace
  for (std::size_t i = 0; i < camera_namespaces_.size(); ++i) {
    const auto& ns = camera_namespaces_[i];
    std::string topic;
    if (ns.empty()) {
      topic = "yolo_tracker/raw_poses";
    } else {
      topic = ns + "/yolo_tracker/raw_poses";
    }

    int cam_id = static_cast<int>(i);
    auto sub = this->create_subscription<
        perception_msgs::msg::PoseWithMetaArray>(
        topic, qos,
        [this, cam_id](
            perception_msgs::msg::PoseWithMetaArray::ConstSharedPtr msg) {
          buffer_.update(cam_id, msg);
        });
    subs_.push_back(sub);

    RCLCPP_INFO(this->get_logger(), "Subscribed to %s (cam_id=%d)",
                topic.c_str(), cam_id);
  }

  // Publishers
  pose_pub_ = this->create_publisher<
      perception_msgs::msg::AssociatedPoseArray>("/associated/poses", qos);
  diag_pub_ = this->create_publisher<diagnostic_msgs::msg::DiagnosticArray>(
      "/associated/diagnostics", 10);

  // Timer
  double period_ms = 1000.0 / association_rate_hz_;
  timer_ = this->create_wall_timer(
      std::chrono::duration<double, std::milli>(period_ms),
      std::bind(&AssociatorNode::onAssociationTimer, this));

  RCLCPP_INFO(this->get_logger(),
              "AssociatorNode started: %zu cameras, rate=%.1f Hz",
              camera_namespaces_.size(), association_rate_hz_);
}

bool AssociatorNode::transformPose(
    const geometry_msgs::msg::PoseStamped& in,
    geometry_msgs::msg::PoseStamped& out) const {
  try {
    out = tf_buffer_->transform(in, base_frame_,
                                tf2::durationFromSec(0.05));
    return true;
  } catch (const tf2::TransformException& ex) {
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                         "TF transform failed: %s", ex.what());
    return false;
  }
}

void AssociatorNode::onAssociationTimer() {
  auto now = this->now();
  auto detections = buffer_.snapshot(now, max_snapshot_age_ms_);

  if (detections.empty()) {
    return;
  }

  if (detections.size() == 1) {
    handleBypass(detections[0]);
  } else {
    handleMultiCamera(detections);
  }

  id_manager_.pruneStaleObjects(stale_object_timeout_sec_, now);
}

void AssociatorNode::handleBypass(const TimedDetections& det) {
  auto now = this->now();
  perception_msgs::msg::AssociatedPoseArray out_msg;
  out_msg.header.stamp = now;
  out_msg.header.frame_id = base_frame_;

  for (const auto& pw : det.msg->poses) {
    geometry_msgs::msg::PoseStamped transformed;
    if (!transformPose(pw.pose, transformed)) continue;

    Eigen::Vector3d center(transformed.pose.position.x,
                           transformed.pose.position.y,
                           transformed.pose.position.z);

    int gid = id_manager_.getOrAssignId(center, pw.class_name, now);

    perception_msgs::msg::AssociatedPose ap;
    ap.global_id = gid;
    ap.class_name = pw.class_name;
    ap.pose = transformed;
    ap.confidence = pw.confidence;
    ap.num_observing_cameras = 1;
    ap.source_camera_ids.push_back(det.camera_id);
    ap.source_local_ids.push_back(pw.object_id);
    ap.source_fitness_scores.push_back(pw.fitness_score);

    out_msg.poses.push_back(ap);
  }

  pose_pub_->publish(out_msg);
  publishDiagnostics(1, out_msg.poses.size());
}

void AssociatorNode::handleMultiCamera(
    const std::vector<TimedDetections>& detections) {
  auto now = this->now();

  // Step a: Transform all poses to base_frame.
  // Store transformed data per camera.
  struct TransformedDetection {
    int camera_id;
    int local_idx;  // index within that camera's message
    std::string class_name;
    geometry_msgs::msg::PoseStamped pose;  // in base_frame
    float confidence;
    float fitness_score;
    int object_id;
  };

  // Per-camera list of transformed detections
  std::vector<std::vector<TransformedDetection>> cam_dets;
  cam_dets.reserve(detections.size());

  for (const auto& det : detections) {
    std::vector<TransformedDetection> td_list;
    for (std::size_t j = 0; j < det.msg->poses.size(); ++j) {
      const auto& pw = det.msg->poses[j];
      geometry_msgs::msg::PoseStamped transformed;
      if (!transformPose(pw.pose, transformed)) continue;

      TransformedDetection td;
      td.camera_id = det.camera_id;
      td.local_idx = static_cast<int>(j);
      td.class_name = pw.class_name;
      td.pose = transformed;
      td.confidence = pw.confidence;
      td.fitness_score = pw.fitness_score;
      td.object_id = pw.object_id;
      td_list.push_back(td);
    }
    cam_dets.push_back(std::move(td_list));
  }

  // Assign a unique global index to each detection across all cameras
  // for use in union-find.
  std::vector<TransformedDetection*> all_dets;
  std::vector<int> global_indices;  // global index for each detection
  for (auto& cam : cam_dets) {
    for (auto& td : cam) {
      global_indices.push_back(static_cast<int>(all_dets.size()));
      all_dets.push_back(&td);
    }
  }

  UnionFind uf;

  // Step b-c: For each pair of cameras, build cost matrix and solve.
  // Build offset map: cam_dets[i] starts at offset[i] in all_dets.
  std::vector<int> offsets;
  int offset = 0;
  for (const auto& cam : cam_dets) {
    offsets.push_back(offset);
    offset += static_cast<int>(cam.size());
  }

  for (std::size_t ci = 0; ci < cam_dets.size(); ++ci) {
    for (std::size_t cj = ci + 1; cj < cam_dets.size(); ++cj) {
      const auto& dets_i = cam_dets[ci];
      const auto& dets_j = cam_dets[cj];
      if (dets_i.empty() || dets_j.empty()) continue;

      int rows = static_cast<int>(dets_i.size());
      int cols = static_cast<int>(dets_j.size());
      Eigen::MatrixXd cost_matrix(rows, cols);

      for (int r = 0; r < rows; ++r) {
        for (int c = 0; c < cols; ++c) {
          if (dets_i[static_cast<std::size_t>(r)].class_name !=
              dets_j[static_cast<std::size_t>(c)].class_name) {
            cost_matrix(r, c) = std::numeric_limits<double>::infinity();
          } else {
            const auto& pi = dets_i[static_cast<std::size_t>(r)].pose.pose.position;
            const auto& pj = dets_j[static_cast<std::size_t>(c)].pose.pose.position;
            double dx = pi.x - pj.x;
            double dy = pi.y - pj.y;
            double dz = pi.z - pj.z;
            cost_matrix(r, c) = std::sqrt(dx * dx + dy * dy + dz * dz);
          }
        }
      }

      auto assignments = HungarianSolver::solve(cost_matrix,
                                                 association_distance_threshold_);

      // Unite matched detections in union-find
      for (const auto& a : assignments) {
        int idx_i = offsets[ci] + a.row;
        int idx_j = offsets[cj] + a.col;
        uf.unite(idx_i, idx_j);
      }
    }
  }

  // Step d-e: Group by connected component, assign global IDs.
  // First, ensure all detections are in the union-find
  for (int i = 0; i < static_cast<int>(all_dets.size()); ++i) {
    uf.find(i);  // ensures the element exists
  }

  auto groups = uf.getGroups();

  perception_msgs::msg::AssociatedPoseArray out_msg;
  out_msg.header.stamp = now;
  out_msg.header.frame_id = base_frame_;

  for (const auto& [rep, members] : groups) {
    // Pick the best detection (highest fitness score) for pose
    int best_idx = -1;
    float best_fitness = -1.0f;
    double sum_conf = 0.0;
    std::string class_name;
    std::set<int> observing_cameras;

    for (int idx : members) {
      auto* td = all_dets[static_cast<std::size_t>(idx)];
      if (td->fitness_score > best_fitness) {
        best_fitness = td->fitness_score;
        best_idx = idx;
      }
      sum_conf += static_cast<double>(td->confidence);
      class_name = td->class_name;
      observing_cameras.insert(td->camera_id);
    }

    if (best_idx < 0) continue;

    auto* best_det = all_dets[static_cast<std::size_t>(best_idx)];

    Eigen::Vector3d center(best_det->pose.pose.position.x,
                           best_det->pose.pose.position.y,
                           best_det->pose.pose.position.z);

    int gid = id_manager_.getOrAssignId(center, class_name, now);

    perception_msgs::msg::AssociatedPose ap;
    ap.global_id = gid;
    ap.class_name = class_name;
    ap.pose = best_det->pose;
    ap.confidence =
        static_cast<float>(sum_conf / static_cast<double>(members.size()));
    ap.num_observing_cameras = static_cast<int32_t>(observing_cameras.size());

    for (int idx : members) {
      auto* td = all_dets[static_cast<std::size_t>(idx)];
      ap.source_camera_ids.push_back(td->camera_id);
      ap.source_local_ids.push_back(td->object_id);
      ap.source_fitness_scores.push_back(td->fitness_score);
    }

    out_msg.poses.push_back(ap);
  }

  pose_pub_->publish(out_msg);
  publishDiagnostics(detections.size(), out_msg.poses.size());
}

void AssociatorNode::publishDiagnostics(std::size_t num_cameras,
                                        std::size_t num_objects) {
  diagnostic_msgs::msg::DiagnosticArray diag_msg;
  diag_msg.header.stamp = this->now();

  diagnostic_msgs::msg::DiagnosticStatus status;
  status.name = "cross_camera_associator";
  status.level = diagnostic_msgs::msg::DiagnosticStatus::OK;
  status.message = "Running";

  diagnostic_msgs::msg::KeyValue kv_cams;
  kv_cams.key = "active_cameras";
  kv_cams.value = std::to_string(num_cameras);
  status.values.push_back(kv_cams);

  diagnostic_msgs::msg::KeyValue kv_objs;
  kv_objs.key = "tracked_objects";
  kv_objs.value = std::to_string(num_objects);
  status.values.push_back(kv_objs);

  diagnostic_msgs::msg::KeyValue kv_global;
  kv_global.key = "global_ids_active";
  kv_global.value = std::to_string(id_manager_.size());
  status.values.push_back(kv_global);

  diag_msg.status.push_back(status);
  diag_pub_->publish(diag_msg);
}

}  // namespace perspective_grasp
