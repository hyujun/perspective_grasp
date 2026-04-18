# Build

Covers compiling the colcon workspace and rebuilding Docker images.

## Full workspace build

[build.sh](../build.sh) wraps `colcon build` and phases the compile in 4 steps so inter-package dependencies resolve cleanly. It uses `$(nproc) - 2` parallel workers.

```bash
cd ~/ros2_ws/perspective_ws

# Default: RelWithDebInfo
./src/perspective_grasp/build.sh

# Release
./src/perspective_grasp/build.sh Release

# Clean first (wipes build/install/log)
./src/perspective_grasp/build.sh --clean
./src/perspective_grasp/build.sh --clean Release

source install/setup.bash
```

## Build order

`build.sh` runs these phases in order; each phase re-sources `install/setup.bash` before the next:

| # | Packages | Why |
|---|----------|-----|
| 1 | `perception_msgs` | All packages depend on custom messages/actions |
| 2 | `teaser_icp_hybrid_registrator` | C++ library consumed by `yolo_pcl_cpp_tracker` |
| 3 | `cross_camera_associator`, `pcl_merge_node` | Multi-camera fusion infrastructure |
| 4 | Everything else | Parallel-safe |

## Single-package build

When iterating on one package, skip `build.sh`:

```bash
cd ~/ros2_ws/perspective_ws
colcon build --packages-select <package_name>
source install/setup.bash
```

Examples:

```bash
colcon build --packages-select yolo_pcl_cpp_tracker
colcon build --packages-select pose_filter_cpp
colcon build --packages-select grasp_pose_planner
```

> Only fall back to `build.sh` when you touch `perception_msgs` or `teaser_icp_hybrid_registrator` (everything downstream needs rebuilding), or when you add a new package.

## Tests

Unit tests use `ament_cmake_gtest`. Three test surfaces ship today:

| Phase | Packages | Binaries | Cases | Notes |
|---|---|---:|---:|---|
| Phase 1 | `yolo_pcl_cpp_tracker`, `teaser_icp_hybrid_registrator` | 5 | 29 | Tracker state machine, CAD loader, `pcl_utils`, hybrid registrator, FPFH |
| Phase 2 | `cross_camera_associator`, `pcl_merge_node` | 8 | 69 | Hungarian / union-find / global-id / pose buffer / overlap filter + rclcpp smoke tests for both nodes |
| Infrastructure | `multi_camera_calibration`, `perception_meta_controller`, `perception_debug_visualizer` | 8 | 60 | Pure-logic detail libraries + rclcpp smoke tests for the two nodes |

Other packages (Phase 3 filtering, Phase 4 ML, Phase 5 manipulation) do not yet ship tests.

```bash
cd ~/ros2_ws/perspective_ws

# Phase 1
colcon test --packages-select teaser_icp_hybrid_registrator yolo_pcl_cpp_tracker

# Phase 2 (cross_camera_associator / pcl_merge_node)
colcon test --packages-select cross_camera_associator pcl_merge_node

# Infrastructure (multi_camera_calibration / meta_controller / debug_visualizer)
colcon test --packages-select multi_camera_calibration perception_meta_controller perception_debug_visualizer

# Show per-test output (passes + failures)
colcon test-result --verbose

# Drop into a single binary for gdb / --gtest_filter
./build/yolo_pcl_cpp_tracker/test_pcl_utils --gtest_filter=CropToRoi.*
./build/cross_camera_associator/test_hungarian_solver --gtest_filter=HungarianSolver.NonSquare*
./build/multi_camera_calibration/test_charuco_detector --gtest_filter=CharucoDetectorTest.Detects*
```

Notes:

- Tests do not need a camera, a running ROS graph, or a GPU — they run against synthetic data / committed fixtures (`.pcd`, rendered ChArUco boards) or in-process pub/sub for the smoke tests.
- **Optional-dep gating** uses CMake `target_compile_definitions`, not runtime detection. Tests are reported as compiled-out (not failed) on boxes without the dep:
  - `HybridRegistrator::align()` is behind `HAS_TEASERPP` — install TEASER++ via [scripts/install_dependencies.sh](../scripts/install_dependencies.sh).
  - `JointOptimizer` convergence test is behind `HAS_CERES` — install `libceres-dev` to enable. The stub-mode tests (empty samples, `isAvailable() == false` message) always run.
- **Pure-logic extraction pattern.** Each tested package extracts non-ROS-coupled logic into a `detail/` header + small library so tests can link without spinning a node:
  - `yolo_pcl_cpp_tracker` → `yolo_tracker_utils` (`cad_model_manager.cpp`, `pcl_utils.cpp`)
  - `multi_camera_calibration` → `calibration_lib` (with extracted `detail/pose_converters.hpp`, `detail/pose6d.hpp`)
  - `perception_meta_controller` → `mode_logic` (mode→nodes mapping, visibility tracker) + `meta_controller_core` (node class) + thin `meta_controller_main.cpp`
  - `perception_debug_visualizer` → `overlay` (drawing helpers) + `visualizer_core` (node class) + thin `visualizer_main.cpp`
  - `cross_camera_associator` → `cross_camera_lib` (Hungarian / GlobalIdManager / CameraPoseBuffer) + `cross_camera_node_core` (node class) + thin `associator_main.cpp`
  - `pcl_merge_node` → `pcl_merge_lib` (CloudPreprocessor / OverlapFilter) + `pcl_merge_node_core` (node class) + thin `merge_main.cpp`
  Don't re-inline these sources back into the executable.
- The four rclcpp smoke tests (`test_meta_controller_smoke`, `test_visualizer_smoke`, `test_associator_node_smoke`, `test_merge_node_smoke`) construct the node, drive it via real publishers/services, and spin a `SingleThreadedExecutor` until the expected output appears (with a 2 s deadline).
- **`RCL_ROS_TIME` gotcha.** Any code that feeds a `rclcpp::Time` built from a message `header.stamp` into arithmetic must use `RCL_ROS_TIME`. `rclcpp::Time(int64_t)` defaults to `RCL_SYSTEM_TIME`; subtracting mixed clock types throws. Test helpers pass `RCL_ROS_TIME` explicitly.

## Rebuilding Docker images

The ML containers compile `perception_msgs` at image-build time (via a `msgs-builder` stage inside [docker/Dockerfile](../docker/Dockerfile)). **Any change to `perception_msgs` requires rebuilding the image.**

```bash
cd ~/ros2_ws/perspective_ws/src/perspective_grasp

# Rebuild all service images
docker compose -f docker/docker-compose.yml build

# Rebuild a specific service (each picks up its dedicated stage)
docker compose -f docker/docker-compose.yml build foundationpose  # foundationpose-runtime
docker compose -f docker/docker-compose.yml build sam2            # sam2-runtime
docker compose -f docker/docker-compose.yml build cosypose        # cosypose-runtime (shared)
docker compose -f docker/docker-compose.yml build megapose        # cosypose-runtime (shared with cosypose)
docker compose -f docker/docker-compose.yml build bundlesdf       # bundlesdf-runtime

# Force no-cache (after a messy dependency change)
docker compose -f docker/docker-compose.yml build --no-cache
```

## Code conventions

- **C++ Standard**: C++20 (`cxx_std_20`)
- **Compiler flags**: `-Wall -Wextra -Wpedantic -Wshadow -Wconversion`
- **C++ build system**: `ament_cmake`
- **Python build system**: `ament_cmake_python` (mixed) or `ament_python`
- **Formatting**: clang-format (Google style with modifications)
- **Namespace**: `perspective_grasp`

## Troubleshooting

| Symptom | Fix |
|---------|-----|
| `perception_msgs/msg/...` not found | Build phase 1 first, then re-source `install/setup.bash` |
| TEASER++ link error | Re-run `scripts/install_dependencies.sh`; check `ldconfig -p \| grep teaserpp` |
| GTSAM not found at link time | Package has optional GTSAM — confirm it was intended; re-run dep script |
| `nvidia-container-cli` error on `docker compose build` | `sudo systemctl restart docker`, confirm `nvidia-ctk runtime configure --runtime=docker` ran |
| Stale Docker build after editing a Python node | Rebuild only if deps changed — source code is bind-mounted at runtime |
| Stale Docker build after editing `perception_msgs` | Rebuild with `docker compose build` (msgs are baked into the image) |

## Next

- [running.md](running.md) — launch the pipeline
- [architecture.md](architecture.md) — pipeline / topic / TF reference
