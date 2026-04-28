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
| `output_dir` | `""` (resolved by launch) | Where ChArUco images + collected samples YAML are written. The bundled `calibration_collect.launch.py` injects `<repo>/runtime_outputs/calibration`; override with `$PERSPECTIVE_GRASP_RUNTIME_OUTPUTS_DIR` or by passing `output_dir` directly when launching the node standalone. |

## Output Layout

Calibration artifacts (annotated PNGs, `collected_samples.yaml`,
`calibration_results.yaml`) land under `<repo>/runtime_outputs/calibration/`
by default. The directory is git-ignored (see repo `.gitignore`). For the
offline `calibration_main` executable, pass that same directory as the
positional `<data_dir>` argument.

## Algorithms

1. **ChArUco Detection** — ArUco marker detection + corner interpolation + PnP pose estimation with sub-pixel refinement
2. **Hand-Eye Calibration** — OpenCV `calibrateHandEye()` with Tsai method (solves AX=XB)
3. **Joint Optimization** (optional) — Ceres Solver minimizes reprojection error across all cameras with Huber loss; auto-differentiable cost function

## Dependencies

- **Required**: `rclcpp`, `sensor_msgs`, `geometry_msgs`, `tf2_ros`, `tf2_eigen`, `cv_bridge`, `std_srvs`, OpenCV (4.0+), Eigen3
- **Optional**: Ceres Solver (for joint optimization, controlled by `HAS_CERES` flag)
- **Python**: `rclpy`, `cv_bridge`, `scipy`

## Library targets

`calibration_lib` (`src/{charuco_detector,hand_eye_solver,joint_optimizer}.cpp`) is the
single shared library used by the offline tool and the test binaries. Pure helpers live
in [`include/multi_camera_calibration/detail/`](include/multi_camera_calibration/detail/):

| Header | Contents | Tested in |
|---|---|---|
| `detail/pose_converters.hpp` | `isometryToRvecTvec`, `rvecTvecToIsometry` (Eigen ↔ OpenCV Rodrigues) | `test_pose_converters` |
| `detail/pose6d.hpp` | `Pose6D` 6-DoF [angle_axis, translation] struct (Ceres-independent) | `test_pose_converters` |

## Build

```bash
colcon build --packages-select multi_camera_calibration
```

## Tests

`ament_cmake_gtest` — 29 cases across 4 binaries:

| Binary | Cases | What it covers |
|---|---:|---|
| `test_hand_eye_solver` | 4 | Eye-in-hand / eye-to-hand on synthetic data, sample-count + size-mismatch validation |
| `test_pose_converters` | 13 | Isometry ↔ rvec/tvec round-trips (50 random samples) + `Pose6D::fromIsometry`/`toIsometry` round-trips, axis-angle magnitude, small-angle branch |
| `test_charuco_detector` | 9 | All 17 dictionary names parse (and unknown throws), empty/blank image returns failure, full rendered board detects with low reprojection error, grayscale input, `min_corners` threshold respected |
| `test_joint_optimizer` | 3 (+2 gated) | `isAvailable()` matches build flag, empty samples fail, Ceres-stub message. Convergence + missing-intrinsics tests run when `HAS_CERES` is defined |

```bash
colcon test --packages-select multi_camera_calibration
colcon test-result --verbose

# Filter inside a single binary
./build/multi_camera_calibration/test_charuco_detector --gtest_filter=CharucoDetectorTest.Detects*
```

The `JointOptimizer` convergence tests use a CMake-time `HAS_CERES` define (set when
`find_package(Ceres QUIET)` succeeds) — they are compiled out, not skipped at runtime,
on boxes without `libceres-dev`. Stub-mode tests always run.
