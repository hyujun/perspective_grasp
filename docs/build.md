# Build

Covers compiling the colcon workspace and rebuilding Docker images.

## Full workspace build

[build.sh](../build.sh) wraps `colcon build` and phases the compile in 4 steps so inter-package dependencies resolve cleanly. It uses `$(nproc) - 2` parallel workers.

`build.sh` auto-activates the workspace Python venv at `<workspace>/.venv` if one exists (created by [scripts/install_host.sh](../scripts/install_host.sh) or [scripts/install_dependencies.sh](../scripts/install_dependencies.sh)). This makes `ament_python` scripts stamp venv-python into their shebangs and ensures pip-installed deps like `ultralytics` are on `sys.path` when colcon runs its tests. `--clean` does **not** wipe `.venv`.

> Path conventions: `${ROS2_WS}` is **your** colcon workspace root (the directory holding `build/`, `install/`, `log/`, `.venv/`); this repo lives at `${ROS2_WS}/src/perspective_grasp/`. See [installation.md § Workspace layout](installation.md#workspace-layout).

```bash
cd ${ROS2_WS}

# Default: RelWithDebInfo
./src/perspective_grasp/build.sh

# Release
./src/perspective_grasp/build.sh Release

# Clean first (wipes build/install/log — NOT .venv)
./src/perspective_grasp/build.sh --clean
./src/perspective_grasp/build.sh --clean Release

# Runtime sourcing: prefer .env.live (sets ROS + workspace + Cyclone DDS in
# one shot). Then layer the venv on top for ultralytics-backed nodes.
source src/perspective_grasp/.env.live
source .venv/bin/activate
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

When iterating on one package, skip `build.sh`. Source `.env.live` for ROS + Cyclone DDS, then layer the venv on top so `ament_python` scripts pick up venv-python as their interpreter:

```bash
cd ${ROS2_WS}
source src/perspective_grasp/.env.live   # ROS + workspace (if built) + Cyclone DDS
source .venv/bin/activate                # ultralytics & friends
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

Unit tests use `ament_cmake_gtest`. Four test surfaces ship today:

| Phase | Packages | Binaries | Cases | Notes |
|---|---|---:|---:|---|
| Phase 1 | `yolo_pcl_cpp_tracker`, `teaser_icp_hybrid_registrator` | 5 | 29 | Tracker state machine, CAD loader, `pcl_utils`, hybrid registrator, FPFH |
| Phase 2 | `cross_camera_associator`, `pcl_merge_node` | 8 | 69 | Hungarian / union-find / global-id / pose buffer / overlap filter + rclcpp smoke tests for both nodes |
| Phase 3 | `pose_filter_cpp`, `pose_graph_smoother` | 4 | 46 | SE(3) IEKF algorithm (predict/update/outlier/10k-cycle long-run) + rclcpp smoke tests (diagnostics, lifecycle, upstream-frame preservation) |
| Infrastructure | `multi_camera_calibration`, `perception_meta_controller`, `perception_debug_visualizer` | 8 | 60 | Pure-logic detail libraries + rclcpp smoke tests for the two nodes |

Phase 4 ML nodes and Phase 5 manipulation do not yet ship tests.

```bash
cd ${ROS2_WS}

# Phase 1
colcon test --packages-select teaser_icp_hybrid_registrator yolo_pcl_cpp_tracker

# Phase 2 (cross_camera_associator / pcl_merge_node)
colcon test --packages-select cross_camera_associator pcl_merge_node

# Phase 3 (pose_filter_cpp / pose_graph_smoother)
colcon test --packages-select pose_filter_cpp pose_graph_smoother

# Infrastructure (multi_camera_calibration / meta_controller / debug_visualizer)
colcon test --packages-select multi_camera_calibration perception_meta_controller perception_debug_visualizer

# Show per-test output (passes + failures)
colcon test-result --verbose

# Drop into a single binary for gdb / --gtest_filter
./build/yolo_pcl_cpp_tracker/test_pcl_utils --gtest_filter=CropToRoi.*
./build/cross_camera_associator/test_hungarian_solver --gtest_filter=HungarianSolver.NonSquare*
./build/pose_filter_cpp/test_iekf_se3 --gtest_filter=IekfSe3Stability.*
./build/multi_camera_calibration/test_charuco_detector --gtest_filter=CharucoDetectorTest.Detects*
```

Notes:

- Tests do not need a camera, a running ROS graph, or a GPU — they run against synthetic data / committed fixtures (`.pcd`, rendered ChArUco boards) or in-process pub/sub for the smoke tests.
- **Optional-dep gating** uses CMake `target_compile_definitions`, not runtime detection. Tests are reported as compiled-out (not failed) on boxes without the dep:
  - `HybridRegistrator::align()` is behind `HAS_TEASERPP` — install TEASER++ via [scripts/install_dependencies.sh](../scripts/install_dependencies.sh).
  - `JointOptimizer` convergence test is behind `HAS_CERES` — install `libceres-dev` to enable. The stub-mode tests (empty samples, `isAvailable() == false` message) always run.
  - `pose_graph_smoother` auto-detects GTSAM via `find_package(GTSAM QUIET)`. When present it defines `HAS_GTSAM` and links `gtsam`; the optimizer is not implemented yet, so the callback stays passthrough in both build paths and the smoke tests are **not** gated on `HAS_GTSAM`.
- **Pure-logic extraction pattern.** Each tested package extracts non-ROS-coupled logic into a `detail/` header + small library so tests can link without spinning a node:
  - `yolo_pcl_cpp_tracker` → `yolo_tracker_utils` (`cad_model_manager.cpp`, `pcl_utils.cpp`)
  - `multi_camera_calibration` → `calibration_lib` (with extracted `detail/pose_converters.hpp`, `detail/pose6d.hpp`)
  - `perception_meta_controller` → `mode_logic` (mode→nodes mapping, visibility tracker) + `meta_controller_core` (node class) + thin `meta_controller_main.cpp`
  - `perception_debug_visualizer` → `overlay` (drawing helpers) + `visualizer_core` (node class) + thin `visualizer_main.cpp`
  - `cross_camera_associator` → `cross_camera_lib` (Hungarian / GlobalIdManager / CameraPoseBuffer) + `cross_camera_node_core` (node class) + thin `associator_main.cpp`
  - `pcl_merge_node` → `pcl_merge_lib` (CloudPreprocessor / OverlapFilter) + `pcl_merge_node_core` (node class) + thin `merge_main.cpp`
  - `pose_filter_cpp` → `iekf_se3_lib` (SE(3) IEKF algorithm) + `pose_filter_node_core` (node class) + thin `pose_filter_main.cpp`
  - `pose_graph_smoother` → `smoother_node_core` (node class) + thin `smoother_main.cpp`
  Don't re-inline these sources back into the executable.
- The six rclcpp smoke tests (`test_meta_controller_smoke`, `test_visualizer_smoke`, `test_associator_node_smoke`, `test_merge_node_smoke`, `test_pose_filter_node_smoke`, `test_smoother_node_smoke`) construct the node, drive it via real publishers/services, and spin a `SingleThreadedExecutor` until the expected output appears (with a 2 s deadline). The smoother smoke test also drives the full lifecycle (`configure → activate → deactivate → cleanup`).
- **`RCL_ROS_TIME` gotcha.** Any code that feeds a `rclcpp::Time` built from a message `header.stamp` into arithmetic must use `RCL_ROS_TIME`. `rclcpp::Time(int64_t)` defaults to `RCL_SYSTEM_TIME`; subtracting mixed clock types throws. Test helpers pass `RCL_ROS_TIME` explicitly.

## Rebuilding Docker images

The ML containers compile `perception_msgs` at image-build time (via a `msgs-builder` stage inside [docker/Dockerfile](../docker/Dockerfile)). **Any change to `perception_msgs` requires rebuilding the image.**

```bash
cd ${ROS2_WS}/src/perspective_grasp

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
| `perception_launch_utils` not found inside container | Rebuild — the `msgs-builder` stage bakes it in. (Touched by commit `11e9b54` for live SAM2.) |
| Container starts but Cyclone fails with "can't open configuration file" | Host `CYCLONEDDS_URI` leaked in. Source `.env.live` instead of exporting manually; compose hardcodes the in-container path. See [debugging.md § 4.9](debugging.md#49-phase-4-node-runs-but-host-doesnt-see-its-topics). |

## Next

- [running.md](running.md) — launch the pipeline
- [architecture.md](architecture.md) — pipeline / topic / TF reference
