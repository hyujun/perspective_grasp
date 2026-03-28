# bundlesdf_unknown_tracker

ROS 2 LifecycleNode for 6D pose estimation and 3D reconstruction of unknown objects using BundleSDF.

## Overview

Tracks and reconstructs unknown objects (no CAD model required) using neural implicit surface representations (SDF). Combines RGB-D frames with instance segmentation masks from SAM2 to jointly optimize object geometry and 6D pose through bundle adjustment.

**Status**: Stub implementation — lifecycle infrastructure is in place, BundleSDF algorithm integration is pending.

## Node

**Node Name**: `bundlesdf_tracker` (LifecycleNode)

| | |
|---|---|
| **Subscribes** | `/camera/color/image_raw` (`Image`), `/camera/depth/image_rect_raw` (`Image`), `/sam2/masks` (`SegmentationArray`) |
| **Publishes** | `/bundlesdf/raw_poses` (`PoseWithMetaArray`) |

## Parameters

| Parameter | Default | Description |
|-----------|---------|-------------|
| `near_plane` | `0.1` | Depth rendering near plane (m) |
| `far_plane` | `2.0` | Depth rendering far plane (m) |
| `sdf_resolution` | `128` | Voxel grid resolution for SDF |
| `keyframe_interval` | `5` | Frames between keyframe selections |
| `bundle_adjust_iterations` | `10` | Bundle adjustment optimization iterations |

## Algorithm (Planned)

BundleSDF represents object geometry as a continuous signed distance field parameterized by a neural network. It jointly optimizes camera/object poses and shape codes over keyframes through photometric and depth alignment losses.

## Dependencies

- `rclpy`, `sensor_msgs`, `geometry_msgs`, `perception_msgs`

## Build

```bash
colcon build --packages-select bundlesdf_unknown_tracker
```
