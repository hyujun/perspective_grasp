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
- Detection bounding box + class label overlays (drawn from cached `/yolo/detections`, clipped to image bounds)
- Pose axis visualization (planned)

## Dependencies

- `rclcpp`, `sensor_msgs`, `geometry_msgs`, `cv_bridge`, `image_transport`, `perception_msgs`, OpenCV

## Library targets

| Target | Source | Purpose |
|---|---|---|
| `overlay` | `src/overlay.cpp` + [`detail/overlay.hpp`](include/perception_debug_visualizer/detail/overlay.hpp) | Pure OpenCV drawing helpers: `draw_mode_overlay`, `draw_detections` (no rclcpp dep) |
| `visualizer_core` | `src/visualizer_node.cpp` | `VisualizerNode` class, links `overlay` |
| `visualizer_node` (exe) | `src/visualizer_main.cpp` | Thin `rclcpp::init` + `spin` wrapper |

## Build

```bash
colcon build --packages-select perception_debug_visualizer
```

## Tests

`ament_cmake_gtest` — 13 cases across 2 binaries:

| Binary | Cases | What it covers |
|---|---:|---|
| `test_overlay` | 11 | `draw_mode_overlay`: empty-image no-op, modifies pixels, paints only top band, green-only color, preserves dims; `draw_detections`: empty array, single bbox paints inside ROI, clamps out-of-bounds bbox, zero-sized bbox skipped, empty image no-op, multiple bboxes |
| `test_visualizer_smoke` | 2 | Node constructs, image republished to `/debug/image` carries the mode overlay (verified via in-process pub/sub) |

```bash
colcon test --packages-select perception_debug_visualizer
colcon test-result --verbose
```
