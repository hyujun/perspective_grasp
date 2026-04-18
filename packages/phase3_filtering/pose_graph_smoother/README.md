# pose_graph_smoother

Sliding-window pose graph optimization for temporal pose smoothing. Currently operates as a
**passthrough** LifecycleNode until the GTSAM optimizer is wired up.

## Overview

Subscribes to the IEKF output, republishes it on the smoothed-poses topic, and broadcasts a TF
per object. When the GTSAM backend is built and implemented, the same node will apply a
sliding-window optimization before republishing.

## Node

**Node Name**: `pose_graph_smoother` (LifecycleNode)

| | |
|---|---|
| **Subscribes** | `/pose_filter/filtered_poses` (`PoseWithMetaArray`) |
| **Publishes**  | `/smoother/smoothed_poses` (`PoseWithMetaArray`, lifecycle publisher) |
| **TF Broadcast** | `object_{id}_smoothed` with parent frame copied from the upstream header |

The TF parent frame is **copied verbatim** from the incoming message's `header.frame_id` so the
multi-camera convention (poses in `ur5e_base_link`) is preserved. `fallback_parent_frame` is
used only when the upstream header has no `frame_id`.

## Parameters

### Always declared

| Parameter | Default | Description |
|-----------|---------|-------------|
| `fallback_parent_frame` | `camera_color_optical_frame` | Used only when `msg.header.frame_id` is empty |

### Declared only when built with GTSAM (`-DHAS_GTSAM`)

| Parameter | Default | Description |
|-----------|---------|-------------|
| `window_size` | `20` | Sliding window size (frames) |
| `prior_noise_pos` | `0.01` | Position noise prior (m) |
| `prior_noise_rot` | `0.05` | Rotation noise prior (rad) |

These are plumbed through `on_configure` but remain unused until the optimizer lands.

## Lifecycle

Standard ROS 2 lifecycle: `configure → activate → deactivate → cleanup`. The publisher is a
`LifecyclePublisher` — deactivated it is silent; the TF broadcaster only fires while active.

## Implementation status

- **Passthrough** in both build paths. The `HAS_GTSAM` branch logs a warning on configure
  (`optimizer not implemented yet, falling back to passthrough`) so downstream consumers stay
  informed and `/smoother/smoothed_poses` keeps flowing.
- `find_package(GTSAM QUIET)` is silent; the package builds with or without GTSAM.

## Dependencies

- `perception_msgs`, `rclcpp`, `rclcpp_lifecycle`, `geometry_msgs`, `tf2_ros`, `tf2_eigen`, Eigen3
- **Optional**: GTSAM (auto-detected; enables `HAS_GTSAM` preprocessor gate)
- **Test-only**: `lifecycle_msgs`

## Build & test

```bash
colcon build --packages-select pose_graph_smoother
colcon test  --packages-select pose_graph_smoother
colcon test-result --verbose
```

## Targets

| Target | Kind | Purpose |
|---|---|---|
| `smoother_node_core` | library | `SmootherNode` class. Smoke tests link this. |
| `smoother_node` | executable | Thin wrapper (`src/smoother_main.cpp`). |

## Tests

- `test_smoother_node_smoke` — 10 smoke tests: construction, parameter read, all 4 lifecycle
  transitions + a full-cycle run, passthrough republish when activated, deactivated-no-publish,
  and **`PreservesUpstreamFrameId`** which guards the fix that the smoother must not overwrite
  the upstream `header.frame_id` (e.g., `ur5e_base_link` in multi-camera setups).
