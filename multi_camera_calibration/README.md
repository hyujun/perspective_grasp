# multi_camera_calibration

Hand-eye calibration for multi-camera robotic systems using ChArUco boards.

## Overview

Provides a complete calibration workflow for determining camera positions relative to a robot base frame. Supports both eye-in-hand (camera on end-effector) and eye-to-hand (fixed camera) configurations with optional joint multi-camera optimization via Ceres Solver.

## Workflow

```
Phase 1: Online Data Collection
  └─ calibration_data_collector node (ChArUco detection + sample capture)

Phase 2: Offline Calibration
  └─ calibration_main executable (hand-eye solving + optional joint optimization)

Phase 3: Static TF Publishing
  └─ generate_static_tf_launch.py (auto-generates launch file for calibrated transforms)
```

## Nodes / Executables

### calibration_data_collector (Python)

| | |
|---|---|
| **Subscribes** | `{camera_ns}/color/image_raw` (`Image`), `/joint_states` (`JointState`) |
| **Service** | `/calibration/capture` (`std_srvs/Trigger`) — triggers sample capture |

### calibration_main (C++)

Offline executable that loads saved data, runs per-camera hand-eye solving, and optionally joint-optimizes all cameras.

```bash
./calibration_main <data_dir> [--use-joint-optimizer]
```

## Parameters

| Parameter | Default | Description |
|-----------|---------|-------------|
| `num_cameras` | `3` | Number of cameras |
| `camera_namespaces` | `["/cam0", "/cam1", "/cam2"]` | Per-camera ROS namespaces |
| `camera_types` | `["eye_in_hand", "eye_to_hand", "eye_to_hand"]` | Calibration type per camera |
| `charuco.squares_x` | `7` | ChArUco board width |
| `charuco.squares_y` | `5` | ChArUco board height |
| `charuco.square_length` | `0.04` | Square size (m) |
| `charuco.marker_length` | `0.03` | ArUco marker size (m) |
| `min_samples` | `30` | Minimum calibration samples |

## Algorithms

1. **ChArUco Detection** — ArUco marker detection + corner interpolation + PnP pose estimation with sub-pixel refinement
2. **Hand-Eye Calibration** — OpenCV `calibrateHandEye()` with Tsai method (solves AX=XB)
3. **Joint Optimization** (optional) — Ceres Solver minimizes reprojection error across all cameras with Huber loss; auto-differentiable cost function

## Dependencies

- **Required**: `rclcpp`, `sensor_msgs`, `geometry_msgs`, `tf2_ros`, `tf2_eigen`, `cv_bridge`, `std_srvs`, OpenCV (4.0+), Eigen3
- **Optional**: Ceres Solver (for joint optimization, controlled by `HAS_CERES` flag)
- **Python**: `rclpy`, `cv_bridge`, `scipy`

## Build

```bash
colcon build --packages-select multi_camera_calibration
```
