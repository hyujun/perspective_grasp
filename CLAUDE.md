# perspective_grasp - 6D Pose Estimation & Manipulation System

## Project Overview
RGB-D camera-based 6D pose estimation pipeline + UR5e + 10-DoF hand manipulation.
**17 ROS 2 packages** across 5 phases + debug visualizer + multi-camera support + calibration + bringup.

**Current status**: C++ packages (Phase 1-3, fusion, infra) are implemented. Phase 1 has 29 gtest
unit tests; Phase 2 adds another 69 gtest cases (8 binaries) across `cross_camera_associator` +
`pcl_merge_node`; Phase 3 adds another 46 gtest cases (4 binaries) across `pose_filter_cpp` +
`pose_graph_smoother` (SE(3) IEKF algorithm + rclcpp/lifecycle smoke); the 3 infrastructure
packages add another 60 gtest cases (8 binaries) — pure-logic tests on extracted detail libraries
plus rclcpp smoke tests for `AssociatorNode`, `MergeNode`, `MetaControllerNode`, and
`VisualizerNode`. **All 5 Phase 4 ML nodes** (SAM2,
FoundationPose, CosyPose, MegaPose, BundleSDF) are wired end-to-end (pluggable real+mock
backends, dedicated Docker runtime stages, multi-cam fan-out). SAM2 is live-verified on
hardware; the other four are mock-smoke-tested — live GPU inference is pending user-side
weights + Docker image builds. Phase 5 **grasp_pose_planner** antipodal planner is implemented
(robot-agnostic planner + `Hand10DoF` adapter; real 10-DoF preshape mapping still TODO).

## Directory Layout
```
perspective_grasp/
├── build.sh                      # Phased colcon wrapper
├── docker/                       # Dockerfile + docker-compose.yml (Phase 4 ML stack)
├── scripts/                      # install_host.sh, install_dependencies.sh
├── .dockerignore                 # MUST stay at repo root (build context)
└── packages/                     # colcon recurses; grouped by role
    ├── interfaces/               #   perception_msgs
    ├── bringup/                  #   perception_bringup (system launches + camera_config)
    ├── phase1_perception/        #   yolo_pcl_cpp_tracker, teaser_icp_hybrid_registrator
    ├── phase2_fusion/            #   cross_camera_associator, pcl_merge_node
    ├── phase3_filtering/         #   pose_filter_cpp, pose_graph_smoother
    ├── phase4_refinement/        #   foundationpose, megapose, cosypose, sam2, bundlesdf
    ├── phase5_manipulation/      #   grasp_pose_planner
    └── infrastructure/           #   meta_controller, debug_visualizer, multi_camera_calibration
```

## Architecture Principle
**Vision Push, Controller Pull.** Vision broadcasts TF2 continuously; Controller does `lookupTransform()` at its own rate.
Heavy ops (Scene analysis, Grasp planning) use ROS 2 Action Servers to avoid blocking the control loop.

## Host + Docker Hybrid
- **Host runs**: C++ nodes, camera drivers, ur5e controller, RViz2, YOLO (Phase 1-3 + fusion + infra)
- **Docker runs**: GPU-heavy Python ML nodes (Phase 4) — dependency isolation for PyTorch/kaolin/nvdiffrast stacks
- Services in `docker/docker-compose.yml`: `foundationpose`, `cosypose`, `megapose`, `sam2`, `bundlesdf` (cosypose and megapose share the `cosypose-runtime` stage)
- Image: `perspective_grasp/ml-base` built from `docker/Dockerfile` (CUDA 12.4 + ROS 2 Jazzy base + compiled `perception_msgs`)
- Compose context = repo root, so volume paths inside `docker-compose.yml` reference `../packages/phase4_refinement/<pkg>`
- `network_mode: host` + `ipc: host` so DDS talks to host nodes directly

## Multi-Camera Support
1~3 cameras supported via config-driven topology. Single YAML defines camera count/type.

- **Config**: `packages/bringup/perception_bringup/config/camera_config.yaml` (3-cam), `camera_config_1cam.yaml`, `camera_config_2cam.yaml`
- **Principle**: N=1 treated as "N=1 multi-camera" (unified code path, no special case)
- **Per-camera**: Nodes namespaced as `/cam0/`, `/cam1/`, `/cam2/`
- **Fusion**: `cross_camera_associator` matches objects across cameras (Hungarian + Union-Find)
- **Point cloud merge**: `pcl_merge_node` merges eye-to-hand depth clouds
- **Calibration**: `multi_camera_calibration` with ChArUco + optional Ceres joint optimization

## Install
- `scripts/install_host.sh` — fresh Ubuntu 24.04: NVIDIA driver 560 + CUDA 12.4 + ROS 2 Jazzy + deps + Docker
- `scripts/install_dependencies.sh` — ROS 2 Jazzy already present, installs remaining C++ / Python deps only
- `COMPOSE_BAKE=true docker compose -f docker/docker-compose.yml build` — builds Phase 4 images. `COMPOSE_BAKE=true` parallelizes the four runtime stages via `buildx bake` and shares cache; the Dockerfile uses BuildKit cache mounts for apt + pip so re-builds skip torch/kaolin/nvdiffrast wheel downloads. Drop the env var only if BuildKit is unavailable.

## Build

```bash
# Full build (respects dependency order)
cd /home/junho/ros2_ws/perspective_ws
./src/perspective_grasp/build.sh               # RelWithDebInfo
./src/perspective_grasp/build.sh Release       # Release
./src/perspective_grasp/build.sh --clean       # Wipe build/install/log first

# Single package
colcon build --packages-select <package_name>

# Source after build
source install/setup.bash

# Run Phase 1 unit tests (29 tests; TEASER++ align tests skip if unavailable)
colcon test --packages-select teaser_icp_hybrid_registrator yolo_pcl_cpp_tracker
colcon test-result --verbose

# Run Phase 2 unit tests (69 tests across Hungarian / union-find / global-id / overlap filter + rclcpp smoke)
colcon test --packages-select cross_camera_associator pcl_merge_node
colcon test-result --verbose

# Run Phase 3 unit tests (46 tests; SE(3) IEKF algorithm + rclcpp/lifecycle smoke)
colcon test --packages-select pose_filter_cpp pose_graph_smoother
colcon test-result --verbose

# Run infrastructure unit tests (60 tests; Ceres-gated optimizer convergence skips if unavailable)
colcon test --packages-select multi_camera_calibration perception_meta_controller perception_debug_visualizer
colcon test-result --verbose
```

`build.sh` uses `$(nproc) - 2` parallel workers and phases the build in 4 steps.

## Build Order
1. `perception_msgs` (all packages depend on this)
2. `teaser_icp_hybrid_registrator` (library, yolo_pcl_cpp_tracker depends on this)
3. `cross_camera_associator` + `pcl_merge_node` (multi-camera infrastructure)
4. All remaining packages (parallel safe)

## Packages (17 total)
Grouped by `packages/<group>/<pkg>/` (colcon discovers recursively).
- **Interfaces**: `perception_msgs` — 12 msgs, 1 srv (`SetMode`), 2 actions (`AnalyzeScene`, `PlanGrasp`)
- **Bringup**: `perception_bringup` — system-level launches (`perception_system.launch.py`, `phase1_bringup.launch.py`) + shared `camera_config*.yaml`
- **Phase 1**: `yolo_pcl_cpp_tracker`, `teaser_icp_hybrid_registrator`
- **Phase 2 (Fusion)**: `cross_camera_associator`, `pcl_merge_node` (69 gtests / 8 binaries; node lib split so smoke tests link class without `main()`)
- **Phase 3 (Filtering)**: `pose_filter_cpp` (SE(3) IEKF, 35 gtests / 2 binaries; Q∝dt unit fix + diagnostics + fitness clamp landed 2026-04-19), `pose_graph_smoother` (optional GTSAM, 11 gtests / 2 binaries; passthrough preserves upstream frame_id — don't overwrite with `fallback_parent_frame`)
- **Phase 4 (Refinement)**: all 5 shipping — `isaac_foundationpose_tracker`, `sam2_instance_segmentor` (live-verified), `cosypose_scene_optimizer` (LifecycleNode + `analyze_scene` action), `megapose_ros2_wrapper`, `bundlesdf_unknown_tracker`. Each has pluggable real+mock backends + dedicated Docker stage + multi-cam fan-out
- **Phase 5 (Manipulation)**: `grasp_pose_planner` — antipodal planner + `Hand10DoF` adapter; finger-joint preshape mapping is a TODO
- **Infra**: `perception_meta_controller`, `perception_debug_visualizer`, `multi_camera_calibration`

## Code Conventions
- **C++ Standard**: C++20 (`cxx_std_20`)
- **Compiler flags**: `-Wall -Wextra -Wpedantic -Wshadow -Wconversion`
- **C++ build**: `ament_cmake`
- **Python build**: `ament_cmake_python` (for mixed packages) or `ament_python`
- **Formatting**: clang-format (Google style with modifications)
- **Namespace**: `perspective_grasp`

## Topic Naming

### Per-camera (namespaced)
- `/{ns}/yolo/detections` (DetectionArray)
- `/{ns}/yolo_tracker/raw_poses` (PoseWithMetaArray)

### Cross-camera fusion
- `/associated/poses` (AssociatedPoseArray) — from cross_camera_associator
- `/merged/points` (PointCloud2) — from pcl_merge_node

### Downstream (global)
- `/pose_filter/filtered_poses` (PoseWithMetaArray)
- `/smoother/smoothed_poses` (PoseWithMetaArray)
- `/foundationpose/raw_poses` (PoseWithMetaArray)
- `/sam2/masks` (SegmentationArray)
- `/cosypose/optimized_poses` (PoseWithMetaArray)
- `/meta_controller/active_pipeline` (PipelineStatus)

## TF2 Frame Convention

### Single camera
- Parent: `camera_color_optical_frame`
- Child: `object_{track_id}_filtered`

### Multi-camera
- Per-camera frames: `cam0_color_optical_frame`, `cam1_color_optical_frame`, `cam2_color_optical_frame`
- All filtered poses in `ur5e_base_link` frame
- Static TF: `ur5e_base_link` → `cam{N}_link` (from calibration)
- Per-camera: `cam{N}_link` → `cam{N}_color_optical_frame` (from camera driver)

## Pipeline Modes (via `perception_meta_controller`)
| Mode | Active nodes |
|------|--------------|
| NORMAL | YOLO tracker + ICP + pose filter |
| HIGH_PRECISION | NORMAL + FoundationPose + pose graph smoother |
| SCENE_ANALYSIS | YOLO tracker + SAM2 + CosyPose + pose filter |

## Key Constraints
- **GPU (dev only)**: This simulation/dev PC has RTX 3070 Ti (8GB VRAM) → during development only YOLO + one GPU-heavy node at a time, mode switching rations VRAM. **Production deployment targets a different, higher-spec PC**, so VRAM is NOT a deployment-time bottleneck and should not drive architectural decisions (e.g., don't reject running multiple ML nodes simultaneously on the grounds that "8GB VRAM can't fit it"). Mode switching exists for dev ergonomics, not as a hard production constraint.
- **No cross-workspace code dependency** with ur5e_ws (communicate via TF2 and actions only).
- **QoS**: BEST_EFFORT + depth 1 for control-path topics.
- **Optional deps**: TEASER++ (ICP fallback), Ceres (OpenCV fallback), GTSAM (passthrough) — packages must build and run sensibly without them.

## AI Harness Notes
Guidance for Claude working in this repo. These supplement the top-level system prompt.

- **Verify before trusting memory.** Memory files under `/home/junho/.claude/projects/-home-junho-ros2-ws-perspective-ws-src-perspective-grasp/memory/` are point-in-time snapshots. Before citing package counts, file paths, or dep status from memory, verify against the current tree (`ls`, `grep`, `git log`).
- **Phase 4 is code-complete.** All 5 nodes (SAM2, FoundationPose, CosyPose, MegaPose, BundleSDF) ship pluggable real+mock backends, Docker stages, and multi-cam fan-out. SAM2 is live-verified; the other 4 are mock-smoke-tested — if the user asks to "implement X", flag that it is already shipping and ask whether they want tuning, live-run help, or a specific feature on top. Real GPU inference is pending user-side Docker image builds + weights/meshes.
- **Host vs Docker.** Before editing a Phase 4 node, remember it runs inside the `ml-base` container. Changes to `perception_msgs` require rebuilding the `msgs-builder` stage — `docker compose build` after editing messages.
- **Build incrementally.** Prefer `colcon build --packages-select <pkg>` over full `build.sh` when iterating on one package — much faster. Only use `build.sh` when dependency order matters (touching `perception_msgs` or `teaser_icp_hybrid_registrator`).
- **Controller workspace is separate.** Do not add code deps on `/home/junho/ros2_ws/ur5e_ws/` — only TF2 frames and action interfaces cross the boundary. If a change seems to require coupling, flag it before implementing.
- **Multi-camera path is unified.** Never add `if N==1` branches in launch files / fusion code — the principle is a single code path parameterized by the camera config.
- **GPU budget is a dev-time concern only.** The 8GB VRAM ceiling applies to this simulation PC, not to the production machine — don't treat it as a hard architectural constraint. When the user asks about running multiple ML nodes together, first clarify dev vs production before arguing VRAM limits.
- **Related skills**: `init` / `claude-md-improver` (maintain this file), `review` / `security-review` (PR review), `simplify` (code review pass), `claude-api` (if touching ML nodes that call hosted models — none do today).
- **Use `Agent` with `Explore` subagent** for broad "where is X used" queries across 17 packages. Direct `Grep`/`Glob` is fine for known symbols.
- **User language**: User writes mixed Korean/English. Mirror the user's language in responses; keep technical identifiers (topic names, class names) in English regardless.

## Workspace Paths
- This repo: `/home/junho/ros2_ws/perspective_ws/src/perspective_grasp/`
- Controller: `/home/junho/ros2_ws/ur5e_ws/src/ur5e-rt-controller/`
- ROS 2: `/opt/ros/jazzy/`
- Memory: `/home/junho/.claude/projects/-home-junho-ros2-ws-perspective-ws-src-perspective-grasp/memory/`
