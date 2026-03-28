# yolo_pcl_cpp_tracker

Hybrid 2D-3D object detection, tracking, and 6D pose estimation pipeline combining YOLO + ByteTrack with ICP-based point cloud registration.

## Overview

This package provides the primary real-time pose estimation pipeline (Phase 1). It integrates a Python-based YOLO detector with a C++ ICP pose estimator to convert RGB-D camera streams into tracked 6D object poses.

## Nodes

### yolo_byte_tracker (Python)

2D object detection and multi-object tracking.

| | |
|---|---|
| **Subscribes** | `/camera/color/image_raw` (`sensor_msgs/Image`) |
| **Publishes** | `/{ns}/yolo/detections` (`perception_msgs/DetectionArray`) |

**Parameters**: `model_path`, `confidence_threshold`, `device`, `track_buffer`, `track_thresh`, `match_thresh`

### pcl_icp_pose_estimator (C++)

3D pose estimation from synchronized detections and point clouds.

| | |
|---|---|
| **Subscribes** | `/{ns}/yolo/detections` (`DetectionArray`), `/camera/depth/points` (`PointCloud2`), `/camera/depth/camera_info` (`CameraInfo`) |
| **Publishes** | `/{ns}/yolo_tracker/raw_poses` (`PoseWithMetaArray`), `/{ns}/yolo_tracker/diagnostics` (`DiagnosticArray`) |
| **TF Broadcast** | `object_{id}` in `camera_color_optical_frame` |

**Parameters**: `icp_max_iterations`, `fitness_threshold`, `voxel_leaf_size`, `outlier_mean_k`, `outlier_stddev`, `max_lost_frames`, `model_dir`

## Data Flow

```
Camera RGB ──→ YOLO + ByteTrack ──→ DetectionArray
                                         │
Camera Depth ────────────────────────────┤ (ApproximateTime sync)
Camera Info  ────────────────────────────┘
                                         ↓
                                  PCL ICP Pose Estimator
                                    ├→ TEASER++ (init/reinit)
                                    └→ ICP (tracking)
                                         ↓
                              PoseWithMetaArray + TF + Diagnostics
```

## Algorithms

- **YOLOv8/v11** — Real-time object detection
- **ByteTrack** — IoU-based multi-object tracking with persistent IDs
- **TEASER++ + ICP** — Hybrid registration via `teaser_icp_hybrid_registrator` library
- **Voxel Grid + Statistical Outlier Removal** — Point cloud preprocessing

## Launch Files

- `tracker.launch.py` — YOLO + ICP (basic)
- `phase1_bringup.launch.py` — YOLO + ICP + Pose Filter
- `perception_system.launch.py` — Full multi-camera system

## Dependencies

- `perception_msgs`, `teaser_icp_hybrid_registrator`
- `rclcpp`, `sensor_msgs`, `geometry_msgs`, `tf2_ros`, `tf2_eigen`, `message_filters`
- PCL, Eigen3, OpenCV
- Python: `ultralytics`, `rclpy`, `cv_bridge`

## Build

```bash
colcon build --packages-select yolo_pcl_cpp_tracker
```
