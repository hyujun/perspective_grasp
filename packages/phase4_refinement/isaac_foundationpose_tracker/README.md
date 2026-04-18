# isaac_foundationpose_tracker

ROS 2 LifecycleNode wrapper for NVIDIA Isaac FoundationPose 6D pose tracker.

## Overview

Integrates FoundationPose, a zero-shot deep learning model for 6D pose estimation, into the perception pipeline. Uses RGB-D images and object meshes for high-accuracy pose tracking without per-object training.

**Status**: Stub implementation — lifecycle infrastructure is in place, core algorithm integration is pending.

## Node

**Node Name**: `foundationpose_tracker` (LifecycleNode)

| | |
|---|---|
| **Subscribes** | `/camera/color/image_raw` (`Image`), `/yolo/detections` (`DetectionArray`) |
| **Publishes** | `/foundationpose/raw_poses` (`PoseWithMetaArray`) |

## Parameters

| Parameter | Default | Description |
|-----------|---------|-------------|
| `mesh_dir` | `""` | Path to object mesh directory |
| `refine_iterations` | `3` | Pose refinement iterations |
| `score_threshold` | `0.5` | Confidence threshold |
| `image_topic` | `/camera/color/image_raw` | RGB image topic |
| `depth_topic` | `/camera/depth/image_rect_raw` | Depth image topic |
| `camera_info_topic` | `/camera/color/camera_info` | Camera intrinsics topic |

## Algorithm (Planned)

FoundationPose combines Vision Transformer feature extraction with iterative geometric refinement for zero-shot 6D pose estimation using 3D object meshes.

## Dependencies

- `rclpy`, `sensor_msgs`, `geometry_msgs`, `perception_msgs`

## Build

```bash
colcon build --packages-select isaac_foundationpose_tracker
```
