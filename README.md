# perspective_grasp

RGB-D camera-based 6D pose estimation pipeline with UR5e + 10-DoF hand manipulation. 17 ROS 2 packages across 5 pipeline phases, supporting 1–3 cameras via config-driven topology.

> **Status.** Phase 1–3 (C++ perception, fusion, filtering) and infra are implemented. Phase 4 ML nodes (FoundationPose, MegaPose, CosyPose, SAM2, BundleSDF) and Phase 5 (`grasp_pose_planner`) ship as **stubs** — launch files and package skeletons, not working inference.

## Documentation

| Doc | Covers |
|-----|--------|
| [docs/installation.md](docs/installation.md) | OS setup, ROS 2 Jazzy, CUDA, TEASER++/GTSAM, Docker |
| [docs/build.md](docs/build.md) | `build.sh`, single-package builds, Docker image rebuilds |
| [docs/running.md](docs/running.md) | Launch by phase, 1/2/3-camera configs, Phase 4 in Docker, pipeline modes |
| [docs/architecture.md](docs/architecture.md) | Topic / TF / QoS reference, design principles |

## Repository layout

```
perspective_grasp/
├── build.sh                      # Phased colcon wrapper
├── docker/                       # Dockerfile + docker-compose.yml (Phase 4 ML stack)
├── scripts/                      # install_host.sh, install_dependencies.sh
├── docs/                         # Installation / build / running / architecture
└── packages/                     # colcon discovers recursively
    ├── interfaces/               # perception_msgs
    ├── bringup/                  # perception_bringup (system launches + camera_config*.yaml)
    ├── phase1_perception/        # yolo_pcl_cpp_tracker, teaser_icp_hybrid_registrator
    ├── phase2_fusion/            # cross_camera_associator, pcl_merge_node
    ├── phase3_filtering/         # pose_filter_cpp, pose_graph_smoother
    ├── phase4_refinement/        # foundationpose, megapose, cosypose, sam2, bundlesdf (stubs)
    ├── phase5_manipulation/      # grasp_pose_planner (stub)
    └── infrastructure/           # meta_controller, debug_visualizer, multi_camera_calibration
```

## Architecture at a glance

**Vision Push, Controller Pull.** Vision broadcasts TF2 continuously; the controller does `lookupTransform()` at its own rate. Heavy ops (scene analysis, grasp planning) are ROS 2 Action Servers so the control loop never blocks. C++ nodes and YOLO run on the host; GPU-heavy Python ML nodes run in Docker containers. See [docs/architecture.md](docs/architecture.md) for the full pipeline diagram.

| Phase | Role | Nodes |
|-------|------|-------|
| 1 | Detection + 6D pose estimation | `yolo_pcl_cpp_tracker`, `teaser_icp_hybrid_registrator` |
| 2 | Multi-camera fusion | `cross_camera_associator`, `pcl_merge_node` |
| 3 | Filtering + smoothing | `pose_filter_cpp`, `pose_graph_smoother` |
| 4 | Refinement (on-demand, stubs) | FoundationPose, MegaPose, CosyPose, SAM2, BundleSDF |
| 5 | Grasp planning (stub) | `grasp_pose_planner` |

## Quick start

```bash
# 1. Clone
mkdir -p ~/ros2_ws/perspective_ws/src
cd ~/ros2_ws/perspective_ws/src
git clone https://github.com/hyujun/perspective_grasp.git

# 2. Install host deps (fresh Ubuntu 24.04)
cd perspective_grasp && ./scripts/install_host.sh
# If ROS 2 Jazzy is already present, use scripts/install_dependencies.sh instead.

# 3. Build the workspace
cd ~/ros2_ws/perspective_ws
./src/perspective_grasp/build.sh
source install/setup.bash

# 4. (Optional) Build Phase 4 Docker images
cd src/perspective_grasp
docker compose -f docker/docker-compose.yml build

# 5. Launch
ros2 launch perception_bringup perception_system.launch.py
```

Full instructions: [docs/installation.md](docs/installation.md) → [docs/build.md](docs/build.md) → [docs/running.md](docs/running.md).

## Prerequisites

- Ubuntu 24.04, NVIDIA GPU (driver 560+, CUDA 12.4+)
- ROS 2 Jazzy Jalisco
- Docker (required only for Phase 4 ML nodes)

## Pipeline modes

Switched at runtime via `perception_meta_controller`:

| Mode | Active nodes |
|------|--------------|
| `NORMAL` | YOLO tracker + ICP + pose filter |
| `HIGH_PRECISION` | `NORMAL` + FoundationPose + pose graph smoother |
| `SCENE_ANALYSIS` | YOLO tracker + SAM2 + CosyPose + pose filter |

## License

Apache-2.0
