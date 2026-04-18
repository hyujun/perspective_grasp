# megapose_ros2_wrapper

ROS 2 LifecycleNode wrapper for MegaPose zero-shot 6D pose estimation.

## Overview

Integrates MegaPose, a deep learning model for estimating 6D object poses from RGB-D images without per-object training. Uses a two-stage coarse-to-refine estimation approach.

**Status**: Stub implementation — lifecycle infrastructure is in place, MegaPose algorithm integration is pending.

## Node

**Node Name**: `megapose_tracker` (LifecycleNode)

| | |
|---|---|
| **Subscribes** | `/camera/color/image_raw` (`Image`), `/yolo/detections` (`DetectionArray`) |
| **Publishes** | `/megapose/raw_poses` (`PoseWithMetaArray`) |

## Parameters

| Parameter | Default | Description |
|-----------|---------|-------------|
| `model_dir` | `""` | Path to MegaPose model weights |
| `n_coarse` | `1` | Coarse estimation iterations |
| `n_refine` | `3` | Refinement iterations |
| `score_threshold` | `0.3` | Confidence threshold |
| `image_topic` | `/camera/color/image_raw` | RGB image topic |
| `depth_topic` | `/camera/depth/image_rect_raw` | Depth image topic |

## Algorithm (Planned)

MegaPose performs zero-shot 6D pose estimation using rendering-to-video matching with a coarse stage for initial hypothesis and an iterative refinement stage for precise alignment.

## Dependencies

- `rclpy`, `sensor_msgs`, `geometry_msgs`, `perception_msgs`

## Build

```bash
colcon build --packages-select megapose_ros2_wrapper
```
