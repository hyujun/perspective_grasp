# pose_filter_cpp

SE(3) Iterated Extended Kalman Filter (IEKF) for multi-source 6D pose fusion.

## Overview

Filters raw 6D object poses from multiple perception sources using an IEKF on the SE(3) manifold. Produces smoothed, covariance-aware pose estimates with outlier rejection. Supports both multi-camera (associated) and legacy single-camera input modes.

## Node

**Node Name**: `pose_filter`

### Associated Mode (Default)

| | |
|---|---|
| **Subscribes** | `/associated/poses` (`AssociatedPoseArray`) |
| **Publishes** | `/pose_filter/filtered_poses` (`PoseWithMetaArray`), `/pose_filter/covariance` (`PoseCovarianceArray`) |
| **TF Broadcast** | `object_{id}_filtered` in configurable parent frame |

### Legacy Mode

Subscribes to 4 raw pose sources: `/yolo_tracker/raw_poses`, `/foundationpose/raw_poses`, `/megapose/raw_poses`, `/bundlesdf/raw_poses`

## Parameters

| Parameter | Default | Description |
|-----------|---------|-------------|
| `process_noise_pos` | `0.001` | Position process noise (m/sqrt(s)) |
| `process_noise_rot` | `0.01` | Rotation process noise (rad/sqrt(s)) |
| `meas_noise_pos` | `0.005` | Measurement noise position (m) |
| `meas_noise_rot` | `0.05` | Measurement noise rotation (rad) |
| `iekf_max_iterations` | `3` | IEKF update iterations |
| `mahalanobis_threshold` | `16.81` | Outlier rejection threshold (chi-squared 6 DOF, p=0.01) |
| `publish_rate_hz` | `30.0` | Output rate |
| `stale_timeout_sec` | `2.0` | Filter pruning timeout |
| `use_associated_input` | `true` | Associated vs legacy mode |

### Source Weights (Legacy Mode)

| Source | Weight | Trust Level |
|--------|--------|-------------|
| `foundationpose` | 0.3 | Highest |
| `megapose` | 0.5 | High |
| `bundlesdf` | 0.8 | Medium |
| `yolo_tracker` | 1.0 | Baseline |

## Algorithm

**12D State**: [rotation(3) + position(3) + angular_velocity(3) + linear_velocity(3)]

- **Prediction**: Constant velocity model on SE(3) using exponential map
- **Update**: Iterated EKF with Lie group innovation (`log(T_meas * T_pred^-1)`)
- **Outlier Rejection**: Mahalanobis distance gating (first iteration)
- **Covariance Update**: Joseph form for numerical stability
- **Dynamic Noise Scaling**: In associated mode, noise scaled by `1/sqrt(num_cameras)` and fitness scores

## Dependencies

- `perception_msgs`, `rclcpp`, `geometry_msgs`, `tf2_ros`, `tf2_eigen`, `diagnostic_msgs`, Eigen3

## Build

```bash
colcon build --packages-select pose_filter_cpp
```
