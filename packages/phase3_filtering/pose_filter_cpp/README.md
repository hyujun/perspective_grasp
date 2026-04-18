# pose_filter_cpp

SE(3) Iterated Extended Kalman Filter (IEKF) for multi-source 6D pose fusion.

## Overview

Filters raw 6D object poses from multiple perception sources using an IEKF on the SE(3) manifold. Produces smoothed, covariance-aware pose estimates with outlier rejection. Supports both multi-camera (associated) and legacy single-camera input modes.

## Node

**Node Name**: `pose_filter`

### Associated Mode (default)

| | |
|---|---|
| **Subscribes** | `/associated/poses` (`AssociatedPoseArray`) |
| **Publishes**  | `/pose_filter/filtered_poses` (`PoseWithMetaArray`), `/pose_filter/covariance` (`PoseCovarianceArray`), `/pose_filter/diagnostics` (`DiagnosticArray`) |
| **TF Broadcast** | `object_{id}_filtered` in `output_frame_id` |

### Legacy Mode

Subscribes to 4 raw pose sources: `/yolo_tracker/raw_poses`, `/foundationpose/raw_poses`, `/megapose/raw_poses`, `/bundlesdf/raw_poses`

## Parameters

### Process noise — continuous-time standard deviations

| Parameter | Default | Description |
|-----------|---------|-------------|
| `process_noise_pos` | `0.001` | Position (m/√s) |
| `process_noise_rot` | `0.01` | Rotation (rad/√s) |
| `process_noise_omega` | `0.1` | Angular velocity (rad/s/√s) |
| `process_noise_vel` | `0.1` | Linear velocity (m/s/√s) |

Discrete-time Q is built as `σ² · dt` per block (not `σ² · dt²`).

### Measurement noise

| Parameter | Default | Description |
|-----------|---------|-------------|
| `meas_noise_pos` | `0.005` | Position (m), scaled per-observation by `noise_scale` |
| `meas_noise_rot` | `0.05` | Rotation (rad), scaled per-observation by `noise_scale` |

### Filter behaviour

| Parameter | Default | Description |
|-----------|---------|-------------|
| `iekf_max_iterations` | `3` | IEKF update iterations (`1` = standard EKF) |
| `mahalanobis_threshold` | `16.81` | Outlier rejection threshold (χ²(6, 0.01)) |
| `publish_rate_hz` | `30.0` | Timer-driven output rate |
| `stale_timeout_sec` | `2.0` | Filter dropped when idle this long |
| `stale_predict_threshold_sec` | `1.0` | If `dt > threshold`, reinit the filter instead of predicting |
| `output_frame_id` | `camera_color_optical_frame` | Parent frame for published poses + TF |
| `use_associated_input` | `true` | `false` selects legacy 4-subscription mode |

### Fitness → noise-scale mapping (associated mode)

`noise_scale` is derived per observation and passed through `IekfSe3::update(meas, noise_scale)`:

1. Start from `1.0`; divide by `√num_observing_cameras` (more cameras → tighter).
2. Multiply by `best_fitness / fitness_reference` (higher fitness → looser).
3. Clamp into `[noise_scale_min, noise_scale_max]`.

| Parameter | Default | Description |
|-----------|---------|-------------|
| `fitness_reference` | `0.001` | Fitness value that maps to `noise_scale = 1` |
| `noise_scale_min` | `0.1` | Lower clamp (tightest) |
| `noise_scale_max` | `10.0` | Upper clamp (loosest) |

### Source weights (legacy mode only)

| Source | Weight | Trust |
|--------|--------|-------|
| `foundationpose` | 0.3 | Highest |
| `megapose` | 0.5 | High |
| `bundlesdf` | 0.8 | Medium |
| `yolo_tracker` | 1.0 | Baseline |

## Algorithm

**12D state**: `[rotation(3), position(3), angular_velocity(3), linear_velocity(3)]`, covariance lives in the tangent space.

- **Prediction**: constant-velocity model on SE(3) via exponential map. `F` linearized at the pre-integration rotation; `Q ∝ dt`.
- **Update**: iterated EKF with Lie group innovation `log(T_meas · T_pred⁻¹)`.
- **Outlier rejection**: Mahalanobis gate on the first iteration.
- **Covariance update**: Joseph form.
- **First `update()` call on an uninitialized filter** auto-initializes via `reset(measurement)` (ignores the outlier gate).

## Diagnostics

`/pose_filter/diagnostics` publishes a `DiagnosticStatus` every tick with:

| Key | Value |
|---|---|
| `active_filters` | current number of per-object IEKF instances |
| `published_filters` | how many of them were published this tick |
| `total_updates` | lifetime accepted + rejected updates |
| `total_rejections` | lifetime Mahalanobis rejections |

Level is `WARN` when `active_filters == 0`, otherwise `OK`.

## Dependencies

- `perception_msgs`, `rclcpp`, `geometry_msgs`, `tf2_ros`, `tf2_eigen`, `diagnostic_msgs`, Eigen3

## Build & test

```bash
colcon build --packages-select pose_filter_cpp
colcon test  --packages-select pose_filter_cpp
colcon test-result --verbose
```

## Targets

| Target | Kind | Purpose |
|---|---|---|
| `iekf_se3_lib` | library | Pure SE(3) IEKF, no ROS. Reusable. Exported. |
| `pose_filter_node_core` | library | `PoseFilterNode` class. Links `iekf_se3_lib`. Smoke tests link this. |
| `pose_filter_node` | executable | Thin wrapper (`src/pose_filter_main.cpp`). |

## Tests

- `test_iekf_se3` — 26 algorithm tests (construction, reset, predict edge cases, Q-scaling linearity, IEKF convergence/iteration, outlier rejection, noise-scale gain, covariance symmetry/PSD, 10k-cycle SO(3) stability at machine precision).
- `test_pose_filter_node_smoke` — 9 rclcpp smoke tests (associated vs legacy construction, all exposed parameters, topic advertisement, diagnostics heartbeat, end-to-end associated → filtered republish).
