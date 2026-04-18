# perception_debug_visualizer

Real-time OpenCV debug overlay for perception pipeline monitoring.

## Overview

Consolidates visualization of object detections, smoothed poses, and pipeline status onto a single annotated debug image stream. Useful for development and troubleshooting of the perception pipeline.

## Node

**Node Name**: `perception_debug_visualizer`

| | |
|---|---|
| **Subscribes** | Image topic (`Image`), `/yolo/detections` (`DetectionArray`), `/smoother/smoothed_poses` (`PoseWithMetaArray`), `/meta_controller/active_pipeline` (`PipelineStatus`) |
| **Publishes** | `/debug/image` (`Image`) |

## Parameters

| Parameter | Default | Description |
|-----------|---------|-------------|
| `image_topic` | `/camera/color/image_raw` | Input camera image topic |

## Visualization Features

- Pipeline mode text overlay (active mode display)
- Detection bounding box overlays (planned)
- Pose axis visualization (planned)

## Dependencies

- `rclcpp`, `sensor_msgs`, `geometry_msgs`, `cv_bridge`, `image_transport`, `perception_msgs`, OpenCV

## Build

```bash
colcon build --packages-select perception_debug_visualizer
```
