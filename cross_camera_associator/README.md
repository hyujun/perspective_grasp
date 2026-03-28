# cross_camera_associator

Multi-camera object association with globally consistent ID management.

## Overview

Receives per-camera object detections, transforms them into a common reference frame, and matches detections across cameras using the Hungarian algorithm and Union-Find. Assigns persistent global IDs that are temporally consistent across frames.

## Node

**Node Name**: `cross_camera_associator`

| | |
|---|---|
| **Subscribes** | `{camera_ns}/yolo_tracker/raw_poses` (`PoseWithMetaArray`) — one per camera |
| **Publishes** | `/associated/poses` (`AssociatedPoseArray`), `/associated/diagnostics` (`DiagnosticArray`) |

## Parameters

| Parameter | Default | Description |
|-----------|---------|-------------|
| `camera_namespaces` | `["/cam0", "/cam1", "/cam2"]` | ROS namespaces for camera sources |
| `base_frame` | `ur5e_base_link` | Common reference frame for association |
| `association_distance_threshold` | `0.05` | Max distance (m) to match detections |
| `max_snapshot_age_ms` | `50.0` | Max age (ms) of data included in association |
| `association_rate_hz` | `30.0` | Association pipeline frequency |
| `stale_object_timeout_sec` | `3.0` | Timeout before pruning lost objects |

## Algorithms

1. **TF2 Transformation** — All poses transformed to `base_frame` before association
2. **Hungarian Algorithm** (Kuhn-Munkres) — O(n^3) optimal assignment between camera pairs using 3D Euclidean distance cost matrix; class-name mismatch = infinite cost
3. **Union-Find** — Groups matched detections across multiple camera pairs into connected components (same physical object)
4. **Global ID Manager** — Assigns persistent IDs by matching new groups to known objects by proximity; prunes stale objects

## Single-Camera Bypass

When only one camera has recent data, the node skips Hungarian/Union-Find and directly assigns global IDs.

## Dependencies

- `perception_msgs`, `rclcpp`, `tf2_ros`, `tf2_eigen`, `tf2_geometry_msgs`, `diagnostic_msgs`, Eigen3

## Build

```bash
colcon build --packages-select cross_camera_associator
```
