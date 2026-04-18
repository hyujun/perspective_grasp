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

## Library targets

The package builds two libraries plus a thin executable so the node class can be linked
into tests without spinning the executable's `main()`:

| Target | Source | Purpose |
|---|---|---|
| `mode_logic` | `src/mode_logic.cpp` + [`detail/mode_logic.hpp`](include/perception_meta_controller/detail/mode_logic.hpp) | Pure-logic helpers: `is_valid_mode`, `compute_active_nodes`, `update_visibility` (all rclcpp-init free) |
| `meta_controller_core` | `src/meta_controller_node.cpp` | `MetaControllerNode` class, links `mode_logic` |
| `meta_controller_node` (exe) | `src/meta_controller_main.cpp` | Thin `rclcpp::init` + `spin` wrapper |

## Build

```bash
colcon build --packages-select perception_meta_controller
```

## Tests

`ament_cmake_gtest` — 18 cases across 2 binaries:

| Binary | Cases | What it covers |
|---|---:|---|
| `test_mode_logic` | 14 | `is_valid_mode` (3 valid, 4 rejects incl. case-sensitivity), `compute_active_nodes` per mode, multi-camera adds `cross_camera_associator` across all modes, zero/negative camera count, visibility map: empty / new / lost / pruning of stale / refresh-prevents-prune / mixed |
| `test_meta_controller_smoke` | 4 | Node constructs, `SetMode` accepts known mode, rejects unknown (and does not change `active_pipeline`), `PipelineStatus` published on timer with `cross_camera_associator` when `num_cameras=2` |

```bash
colcon test --packages-select perception_meta_controller
colcon test-result --verbose
```

The smoke test uses a real `SingleThreadedExecutor` with in-process pubs/subs — it spins
until the expected output appears, with a 2 s deadline.
