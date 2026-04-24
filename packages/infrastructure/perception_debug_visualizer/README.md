# perception_debug_visualizer

Real-time OpenCV debug overlay for perception pipeline monitoring.

## Overview

Consolidates visualization of object detections, smoothed poses, and pipeline status onto a single annotated debug image stream. Useful for development and troubleshooting of the perception pipeline.

Multi-camera aware: subscribes to **all** cameras listed in a `perception_bringup` `camera_config*.yaml` and republishes a single overlayed image for the `active_camera_index` camera. Hot-swap via `ros2 param set`.

**See [`docs/debugging.md`](../../../docs/debugging.md) at the workspace root for launch recipes and a symptom-driven pipeline debugging playbook.**

## Node

**Node Name**: `perception_debug_visualizer`

| | |
|---|---|
| **Subscribes (per-camera)** | `/{ns}/{image_topic_suffix}` (`Image`), `/{ns}/{detection_topic_suffix}` (`DetectionArray`), `/{ns}/{sam2_masks_suffix}` (`SegmentationArray`), `/{ns}/{camera_info_suffix}` (`CameraInfo`) — one set per entry in `camera_namespaces` |
| **Subscribes (global)**     | `/smoother/smoothed_poses` (`PoseWithMetaArray`), `/meta_controller/active_pipeline` (`PipelineStatus`) |
| **Publishes** | `/debug/image` (`Image`) |
| **TF** | Looks up `smoothed_poses.header.frame_id` → active camera's `CameraInfo.header.frame_id` for each frame rendered. |

## Parameters

| Parameter | Default | Description |
|-----------|---------|-------------|
| `camera_namespaces` | `[""]` | Per-camera namespace list. Driven by `camera_config*.yaml` at launch. |
| `image_topic_suffix` | `camera/color/image_raw` | Topic suffix composed with each namespace. |
| `detection_topic_suffix` | `yolo/detections` | Topic suffix composed with each namespace. |
| `sam2_masks_suffix` | `sam2/masks` | Topic suffix for per-camera SAM2 `SegmentationArray`. |
| `camera_info_suffix` | `camera/color/camera_info` | Topic suffix for per-camera `CameraInfo` (intrinsics + optical frame id). |
| `enable_sam2_masks` | `true` | Toggle alpha-blended instance mask overlays. Hot-settable. |
| `mask_alpha` | `0.4` | Mask overlay opacity, clamped to `[0, 1]`. Hot-settable. |
| `enable_pose_axes` | `true` | Toggle projected X/Y/Z axis triads from `/smoother/smoothed_poses`. Hot-settable. |
| `axis_length_m` | `0.05` | Body-frame axis length in meters (must be `> 0`). Hot-settable. |
| `pose_axis_thickness` | `2` | Axis line thickness in pixels, clamped to `[1, 10]`. Hot-settable. |
| `active_camera_index` | `0` | Which camera gets overlayed on `/debug/image`. Hot-swappable. |
| `image_topic` | `""` | Legacy override — if non-empty, replaces camera 0's computed image topic. |

## Visualization Features

- Pipeline mode text overlay (active mode display)
- Detection bounding box + class label overlays (drawn from cached `/yolo/detections`, clipped to image bounds)
- SAM2 instance mask alpha-blend overlay (palette keyed by `Segmentation::id`, skipped when encoding is not `mono8` or dimensions mismatch)
- Pose-axis triad overlay (X=red, Y=green, Z=blue) from `/smoother/smoothed_poses` — TF-projected into the active camera's optical frame via `CameraInfo::k`, `track_id` stamped near each origin

## Dependencies

- `rclcpp`, `sensor_msgs`, `geometry_msgs`, `cv_bridge`, `image_transport`, `perception_msgs`, `tf2`, `tf2_ros`, `tf2_geometry_msgs`, OpenCV

## Library targets

| Target | Source | Purpose |
|---|---|---|
| `overlay` | `src/overlay.cpp` + [`detail/overlay.hpp`](include/perception_debug_visualizer/detail/overlay.hpp) | Pure OpenCV drawing helpers: `draw_mode_overlay`, `draw_detections`, `draw_masks`, `project_pose_axes`, `draw_pose_axes` (no rclcpp/TF dep) |
| `visualizer_core` | `src/visualizer_node.cpp` | `VisualizerNode` class, links `overlay` and owns TF buffer + per-camera `CameraInfo` cache |
| `visualizer_node` (exe) | `src/visualizer_main.cpp` | Thin `rclcpp::init` + `spin` wrapper |

## Build

```bash
colcon build --packages-select perception_debug_visualizer
```

## Tests

`ament_cmake_gtest` — 36 cases across 2 binaries:

| Binary | Cases | What it covers |
|---|---:|---|
| `test_overlay` | 30 | `draw_mode_overlay` (5): empty-image no-op, modifies pixels, paints only top band, green-only color, preserves dims; `draw_detections` (6): empty array, single bbox paints inside ROI, clamps out-of-bounds bbox, zero-sized bbox skipped, empty image no-op, multiple bboxes; `draw_masks` (9): empty array, empty image, mask-boundary respect, dimension mismatch skipped, unsupported encoding skipped, alpha=0 no-op, alpha=1 full replace, different ids → different palette, negative id no-crash; `project_pose_axes` (5): identity pose hits principal point, track id carried, origin behind camera rejected, invalid inputs rejected, 90° yaw swaps X-end; `draw_pose_axes` (5): empty image no-op, empty vector no-op, paints R/G/B channels, off-image clipLine no-op, multiple triads render |
| `test_visualizer_smoke` | 6 | Node constructs, legacy `image_topic` republish path, multi-camera construction with 3 namespaces, `active_camera_index` runtime update + out-of-range rejection, `compose_topic` helper namespace/suffix joining, pose-axes params hot-toggle + range validation |

```bash
colcon test --packages-select perception_debug_visualizer
colcon test-result --verbose
```
