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

## Library targets

The package ships two libraries plus a thin executable so the node class can be linked into the
smoke test without spinning the executable's `main()`:

| Target | Source | Purpose |
|---|---|---|
| `cross_camera_lib` | `hungarian_solver.cpp`, `global_id_manager.cpp`, `camera_pose_buffer.cpp` | Pure-logic algorithms + state (rclcpp-init free aside from `rclcpp::Time`) |
| `cross_camera_node_core` | `associator_node.cpp` | `AssociatorNode` class, links `cross_camera_lib` |
| `associator_node` (exe) | `associator_main.cpp` | Thin `rclcpp::init` + `spin` wrapper |

`union_find.hpp` is header-only and not linked into any library.

## Build

```bash
colcon build --packages-select cross_camera_associator
```

## Tests

`ament_cmake_gtest` — 46 cases across 5 binaries:

| Binary | Cases | What it covers |
|---|---:|---|
| `test_hungarian_solver` | 11 | Square/non-square (rows<>cols), 1x1, padding-above-threshold rejection, `isfinite` guard on inf class-mismatch costs, partial threshold keeps diagonal-optimal only, tied costs return valid permutation, random 6x6 sanity, empty + degenerate `0xN` / `Nx0` |
| `test_global_id_manager` | 11 | First id=0, same-pos reuse, different pos → new id, class-name hard filter, closest match wins among candidates, beyond-threshold → new id, prune stale, monotonic `next_id_` across prune, `last_seen` refreshed on reuse, `reset()` restarts counter |
| `test_union_find` | 9 | `find` on unknown auto-creates singleton, disjoint stays disjoint, transitive unite, self-unite no-op, already-connected unite no-op, `getGroups` disjoint-set counting + membership, path compression on 10-node chain |
| `test_camera_pose_buffer` | 7 | Empty buffer, update+overwrite same cam_id, `max_age_ms` filter, multi-camera fan-out, negative-age (future timestamp) exclusion, concurrent writers smoke |
| `test_associator_node_smoke` | 3 | 1-cam + 3-cam construction, `/associated/poses` advertised; links `cross_camera_node_core` and drives a `SingleThreadedExecutor` |

```bash
colcon test --packages-select cross_camera_associator
colcon test-result --verbose

# Drop into a single binary for gdb / --gtest_filter
./build/cross_camera_associator/test_hungarian_solver --gtest_filter=HungarianSolver.NonSquare*
```

> **Gotcha.** `CameraPoseBuffer` wraps `msg->header.stamp` as `RCL_ROS_TIME`. When constructing
> `rclcpp::Time` in tests, always pass `RCL_ROS_TIME` explicitly — the default is
> `RCL_SYSTEM_TIME` and subtracting mismatched clocks throws.
