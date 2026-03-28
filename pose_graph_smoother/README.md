# pose_graph_smoother

Sliding-window pose graph optimization for temporal pose smoothing.

## Overview

A LifecycleNode that smooths object pose estimates using pose graph optimization. Currently operates in **passthrough mode** (republishes filtered poses as smoothed), with planned GTSAM integration for full optimization.

## Node

**Node Name**: `pose_graph_smoother` (LifecycleNode)

| | |
|---|---|
| **Subscribes** | `/pose_filter/filtered_poses` (`PoseWithMetaArray`) |
| **Publishes** | `/smoother/smoothed_poses` (`PoseWithMetaArray`) |
| **TF Broadcast** | `object_{id}` in configurable camera frame |

## Parameters

| Parameter | Default | Description |
|-----------|---------|-------------|
| `window_size` | `20` | Sliding window size (frames) |
| `prior_noise_pos` | `0.01` | Position noise prior (m) |
| `prior_noise_rot` | `0.05` | Rotation noise prior (rad) |
| `camera_frame_id` | `camera_color_optical_frame` | Reference frame for TF |

## Implementation Status

- **Without GTSAM**: Passthrough mode (republishes input directly)
- **With GTSAM**: Sliding-window pose graph optimization (planned, conditional compilation via `HAS_GTSAM`)

## Dependencies

- `perception_msgs`, `rclcpp`, `rclcpp_lifecycle`, `geometry_msgs`, `tf2_ros`, `tf2_eigen`, Eigen3
- **Optional**: GTSAM (for full smoothing implementation)

## Build

```bash
colcon build --packages-select pose_graph_smoother
```
