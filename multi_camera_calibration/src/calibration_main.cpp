// Copyright 2026 perspective_grasp
// SPDX-License-Identifier: Apache-2.0

/// Offline calibration executable.
/// Loads saved calibration data from YAML, runs hand-eye solver
/// (and optionally joint optimizer), and saves results.

#include "multi_camera_calibration/hand_eye_solver.hpp"
#include "multi_camera_calibration/joint_optimizer.hpp"

#include <Eigen/Geometry>
#include <opencv2/core.hpp>

#include <rclcpp/rclcpp.hpp>

#include <filesystem>
#include <fstream>
#include <iostream>
#include <map>
#include <string>
#include <vector>

namespace fs = std::filesystem;

namespace {

/// Parse an Isometry3d from a YAML FileNode containing "rotation" (3x3) and
/// "translation" (3x1) entries stored as flat lists.
Eigen::Isometry3d parseIsometry(const cv::FileNode& node) {
  Eigen::Isometry3d T = Eigen::Isometry3d::Identity();

  std::vector<double> rot_vals, trans_vals;
  node["rotation"] >> rot_vals;
  node["translation"] >> trans_vals;

  if (rot_vals.size() == 9 && trans_vals.size() == 3) {
    Eigen::Matrix3d R;
    for (int r = 0; r < 3; ++r) {
      for (int c = 0; c < 3; ++c) {
        R(r, c) = rot_vals[static_cast<size_t>(r * 3 + c)];
      }
    }
    T.linear() = R;
    T.translation() = Eigen::Vector3d(trans_vals[0], trans_vals[1], trans_vals[2]);
  }
  return T;
}

/// Write an Isometry3d to a YAML FileStorage.
void writeIsometry(cv::FileStorage& fs_out, const std::string& name,
                   const Eigen::Isometry3d& T) {
  fs_out << name << "{";
  std::vector<double> rot_vals(9), trans_vals(3);
  for (int r = 0; r < 3; ++r) {
    for (int c = 0; c < 3; ++c) {
      rot_vals[static_cast<size_t>(r * 3 + c)] = T.rotation()(r, c);
    }
  }
  trans_vals[0] = T.translation().x();
  trans_vals[1] = T.translation().y();
  trans_vals[2] = T.translation().z();

  fs_out << "rotation" << rot_vals;
  fs_out << "translation" << trans_vals;
  fs_out << "}";
}

struct CameraConfig {
  int id = 0;
  std::string type;  // "eye_in_hand" or "eye_to_hand"
};

}  // namespace

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto logger = rclcpp::get_logger("calibration_main");

  // Parse command-line arguments
  if (argc < 2) {
    RCLCPP_ERROR(logger,
                 "Usage: calibration_main <data_dir> [--use-joint-optimizer]");
    RCLCPP_ERROR(logger,
                 "  data_dir: directory containing calibration_data.yaml");
    return 1;
  }

  std::string data_dir = argv[1];
  bool use_joint_optimizer = false;
  for (int i = 2; i < argc; ++i) {
    if (std::string(argv[i]) == "--use-joint-optimizer") {
      use_joint_optimizer = true;
    }
  }

  // Load calibration data
  std::string data_file = data_dir + "/calibration_data.yaml";
  if (!fs::exists(data_file)) {
    RCLCPP_ERROR(logger, "Calibration data file not found: %s",
                 data_file.c_str());
    return 1;
  }

  cv::FileStorage fs_in(data_file, cv::FileStorage::READ);
  if (!fs_in.isOpened()) {
    RCLCPP_ERROR(logger, "Failed to open: %s", data_file.c_str());
    return 1;
  }

  // Read camera configurations
  int num_cameras = 0;
  fs_in["num_cameras"] >> num_cameras;
  RCLCPP_INFO(logger, "Number of cameras: %d", num_cameras);

  std::vector<CameraConfig> camera_configs;
  cv::FileNode cameras_node = fs_in["cameras"];
  for (auto it = cameras_node.begin(); it != cameras_node.end(); ++it) {
    CameraConfig cfg;
    (*it)["id"] >> cfg.id;
    (*it)["type"] >> cfg.type;
    camera_configs.push_back(cfg);
  }

  // Read samples per camera
  // Structure: samples -> [{ camera_id, T_robot, T_target_cam }]
  std::map<int, std::vector<Eigen::Isometry3d>> robot_poses;     // T_base_gripper
  std::map<int, std::vector<Eigen::Isometry3d>> target_cam_poses; // T_target_cam

  cv::FileNode samples_node = fs_in["samples"];
  int total_samples = 0;
  for (auto it = samples_node.begin(); it != samples_node.end(); ++it) {
    int camera_id = 0;
    (*it)["camera_id"] >> camera_id;

    Eigen::Isometry3d T_robot = parseIsometry((*it)["T_robot"]);
    Eigen::Isometry3d T_target = parseIsometry((*it)["T_target_cam"]);

    robot_poses[camera_id].push_back(T_robot);
    target_cam_poses[camera_id].push_back(T_target);
    ++total_samples;
  }

  fs_in.release();
  RCLCPP_INFO(logger, "Loaded %d total samples", total_samples);

  // Run per-camera hand-eye calibration
  perspective_grasp::HandEyeSolver solver;
  std::map<int, perspective_grasp::CalibrationResult> results;

  for (const auto& cam_cfg : camera_configs) {
    int cam_id = cam_cfg.id;
    auto& rp = robot_poses[cam_id];
    auto& tp = target_cam_poses[cam_id];

    RCLCPP_INFO(logger, "Camera %d (%s): %zu samples", cam_id,
                cam_cfg.type.c_str(), rp.size());

    perspective_grasp::CalibrationResult cal_result;
    if (cam_cfg.type == "eye_in_hand") {
      cal_result = solver.solveEyeInHand(rp, tp);
    } else if (cam_cfg.type == "eye_to_hand") {
      // For eye-to-hand, we pass T_gripper_base (inverse of T_base_gripper)
      std::vector<Eigen::Isometry3d> T_gripper_base;
      T_gripper_base.reserve(rp.size());
      for (const auto& T_bg : rp) {
        T_gripper_base.push_back(T_bg.inverse());
      }
      cal_result = solver.solveEyeToHand(T_gripper_base, tp);
    } else {
      RCLCPP_ERROR(logger, "Unknown camera type: %s", cam_cfg.type.c_str());
      continue;
    }

    if (cal_result.success) {
      results[cam_id] = cal_result;
      Eigen::Vector3d t = cal_result.T_cam_base.translation();
      RCLCPP_INFO(logger, "Camera %d: translation = [%.4f, %.4f, %.4f]",
                  cam_id, t.x(), t.y(), t.z());
    } else {
      RCLCPP_ERROR(logger, "Camera %d calibration FAILED", cam_id);
    }
  }

  // Optionally run joint optimization
  if (use_joint_optimizer) {
    if (!perspective_grasp::JointOptimizer::isAvailable()) {
      RCLCPP_WARN(logger,
                  "Joint optimizer requested but Ceres is not available. "
                  "Skipping joint optimization.");
    } else {
      RCLCPP_INFO(logger, "Running joint optimization...");
      // Joint optimization would require intrinsics and point correspondences
      // which would be loaded from additional data files. For now, log a
      // placeholder message.
      RCLCPP_INFO(logger,
                  "Joint optimization requires point correspondence data. "
                  "Please ensure calibration_data.yaml includes points_3d and "
                  "points_2d per sample.");
    }
  }

  // Save results
  std::string output_file = data_dir + "/calibration_results.yaml";
  cv::FileStorage fs_out(output_file, cv::FileStorage::WRITE);
  if (!fs_out.isOpened()) {
    RCLCPP_ERROR(logger, "Failed to open output file: %s", output_file.c_str());
    return 1;
  }

  fs_out << "num_cameras" << num_cameras;
  fs_out << "results" << "[";
  for (const auto& [cam_id, cal_result] : results) {
    fs_out << "{";
    fs_out << "camera_id" << cam_id;
    fs_out << "success" << static_cast<int>(cal_result.success);
    fs_out << "reprojection_error" << cal_result.reprojection_error;
    writeIsometry(fs_out, "T_cam_base", cal_result.T_cam_base);
    fs_out << "}";
  }
  fs_out << "]";
  fs_out.release();

  RCLCPP_INFO(logger, "Results saved to: %s", output_file.c_str());

  // Print summary to console
  std::cout << "\n=== Calibration Results ===\n";
  for (const auto& [cam_id, cal_result] : results) {
    Eigen::Vector3d t = cal_result.T_cam_base.translation();
    Eigen::Quaterniond q(cal_result.T_cam_base.rotation());
    std::cout << "Camera " << cam_id << ":\n"
              << "  Translation: [" << t.x() << ", " << t.y() << ", " << t.z()
              << "]\n"
              << "  Quaternion (xyzw): [" << q.x() << ", " << q.y() << ", "
              << q.z() << ", " << q.w() << "]\n"
              << "  Success: " << (cal_result.success ? "yes" : "no") << "\n\n";
  }

  rclcpp::shutdown();
  return 0;
}
