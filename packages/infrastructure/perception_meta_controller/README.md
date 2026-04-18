# perception_meta_controller

State machine orchestrator for dynamically switching perception pipeline modes.

## Overview

Central control point that manages three perception pipeline modes, monitors object tracking health, and publishes real-time pipeline status. Provides a ROS 2 service for runtime mode switching.

## Node

**Node Name**: `perception_meta_controller`

| | |
|---|---|
| **Subscribes** | `/associated/poses` (`AssociatedPoseArray`) |
| **Publishes** | `/meta_controller/active_pipeline` (`PipelineStatus`) |
| **Service** | `/meta_controller/set_mode` (`SetMode`) |

## Pipeline Modes

| Mode | Active Nodes | Use Case |
|------|-------------|----------|
| **NORMAL** | YOLO tracker, ICP pose estimator, pose filter, (cross-camera associator) | Standard real-time tracking |
| **HIGH_PRECISION** | NORMAL + FoundationPose, pose graph smoother | High-accuracy manipulation tasks |
| **SCENE_ANALYSIS** | YOLO tracker, SAM2, CosyPose optimizer, pose filter, (cross-camera associator) | Scene understanding |

## Parameters

| Parameter | Default | Description |
|-----------|---------|-------------|
| `status_publish_rate_hz` | `1.0` | Status publishing frequency |
| `default_mode` | `NORMAL` | Initial pipeline mode |
| `num_cameras` | `1` | Number of cameras (adds cross-camera associator if > 1) |
| `stale_object_timeout_sec` | `3.0` | Timeout before marking objects as lost |

## Object Tracking

Monitors object visibility by tracking which cameras observe each global object ID. Objects not seen within `stale_object_timeout_sec` are pruned and counted as lost.

## Dependencies

- `rclcpp`, `std_msgs`, `perception_msgs`

## Build

```bash
colcon build --packages-select perception_meta_controller
```
