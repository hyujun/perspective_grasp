# perspective_grasp

RGB-D camera-based 6D pose estimation pipeline with UR5e + 10-DoF hand manipulation. 18 ROS 2 packages across 5 pipeline phases, supporting 1–3 cameras via config-driven topology.

> **Status.** Phase 1–3 (C++ perception, fusion, filtering) and infra are implemented. **All 5 Phase 4 ML nodes** (SAM2, FoundationPose, CosyPose, MegaPose, BundleSDF) are wired end-to-end with pluggable real+mock backends, dedicated Docker runtime stages, and multi-camera fan-out. **RealSense → YOLO → SAM2 is live-verified on hardware** (1-cam D-series, host↔container DDS lockstep via `.env.live`); the other four Phase 4 nodes are mock-smoke-tested — live GPU inference is pending user-side weights + Docker image builds. Phase 5 (`grasp_pose_planner`) ships an antipodal planner + `Hand10DoF` adapter (real 10-DoF preshape mapping still TODO).

## Documentation

| Doc | Covers |
|-----|--------|
| [docs/installation.md](docs/installation.md) | OS setup, ROS 2 Jazzy, CUDA, TEASER++/GTSAM, Docker |
| [docs/build.md](docs/build.md) | `build.sh`, single-package builds, Docker image rebuilds |
| [docs/running.md](docs/running.md) | Launch by phase, 1/2/3-camera configs, Phase 4 in Docker, pipeline modes |
| [docs/architecture.md](docs/architecture.md) | Topic / TF / QoS reference, design principles |
| [docs/debugging.md](docs/debugging.md) | Symptom-driven pipeline debugging playbook (Phase 1–4, debug visualizer) |

## Repository layout

```
perspective_grasp/
├── build.sh                      # Phased colcon wrapper
├── .env.live                     # Host shell env — ROS + workspace + Cyclone DDS
├── docker/                       # Dockerfile + docker-compose.yml (Phase 4 ML stack)
├── scripts/                      # install_host.sh, install_dependencies.sh, install_realsense.sh
├── docs/                         # Installation / build / running / architecture / debugging
└── packages/                     # colcon discovers recursively
    ├── interfaces/               # perception_msgs
    ├── bringup/                  # perception_bringup (system launches + camera_config*.yaml)
    ├── phase1_perception/        # yolo_pcl_cpp_tracker, teaser_icp_hybrid_registrator
    ├── phase2_fusion/            # cross_camera_associator, pcl_merge_node
    ├── phase3_filtering/         # pose_filter_cpp, pose_graph_smoother
    ├── phase4_refinement/        # foundationpose + sam2 + cosypose + megapose + bundlesdf (all shipping)
    ├── phase5_manipulation/      # grasp_pose_planner (antipodal planner + Hand10DoF adapter)
    └── infrastructure/           # meta_controller, debug_visualizer, multi_camera_calibration, perception_launch_utils
```

## Architecture at a glance

**Vision Push, Controller Pull.** Vision broadcasts TF2 continuously; the controller does `lookupTransform()` at its own rate. Heavy ops (scene analysis, grasp planning) are ROS 2 Action Servers so the control loop never blocks. C++ nodes and YOLO run on the host; GPU-heavy Python ML nodes run in Docker containers. See [docs/architecture.md](docs/architecture.md) for the full pipeline diagram.

| Phase | Role | Nodes |
|-------|------|-------|
| 1 | Detection + 6D pose estimation | `yolo_pcl_cpp_tracker`, `teaser_icp_hybrid_registrator` |
| 2 | Multi-camera fusion | `cross_camera_associator`, `pcl_merge_node` |
| 3 | Filtering + smoothing | `pose_filter_cpp`, `pose_graph_smoother` |
| 4 | Refinement (on-demand) | FoundationPose, SAM2, CosyPose, MegaPose, BundleSDF (all shipping — live GPU verification pending on 4/5) |
| 5 | Grasp planning | `grasp_pose_planner` (antipodal + `Hand10DoF`) |

## Quick start

> `${ROS2_WS}` is **your** colcon workspace root — pick anything (`~/ros2_ws/perspective_ws`, `~/dev_ws`, `/opt/work`, …); `build/`, `install/`, `log/`, and `.venv/` will be created here. The only fixed assumption is that this repo is cloned at `${ROS2_WS}/src/perspective_grasp/`. Either `export ROS2_WS=...` once and paste the snippet, or substitute mentally. Details: [docs/installation.md § Workspace layout](docs/installation.md#workspace-layout).

```bash
export ROS2_WS=~/ros2_ws/perspective_ws    # example — use any path you prefer

# 1. Clone
mkdir -p ${ROS2_WS}/src
cd ${ROS2_WS}/src
git clone https://github.com/hyujun/perspective_grasp.git

# 2. Install host deps (fresh Ubuntu 24.04)
cd perspective_grasp && ./scripts/install_host.sh
# If ROS 2 Jazzy is already present, use scripts/install_dependencies.sh instead.
# For Intel RealSense cameras (optional): ./scripts/install_realsense.sh

# 3. Build the workspace (creates ${ROS2_WS}/.venv during install step)
cd ${ROS2_WS}
./src/perspective_grasp/build.sh

# 4. (Optional) Build Phase 4 Docker images
cd src/perspective_grasp
docker compose -f docker/docker-compose.yml build

# 5. Source runtime env + launch
cd ${ROS2_WS}
source src/perspective_grasp/.env.live    # ROS + workspace + Cyclone DDS
source .venv/bin/activate                 # ultralytics & friends
ros2 launch perception_bringup perception_system.launch.py
```

> Two host-side things must be sourced per terminal: `.env.live` (ROS + workspace + Cyclone DDS — needed so host shells match the Phase 4 containers) and `.venv/bin/activate` (host pip deps like `ultralytics`, kept out of `~/.local` / system site-packages). `build.sh` auto-activates the venv during the build itself. Details: [docs/installation.md#host-shell-env---envlive](docs/installation.md#host-shell-env---envlive) and [docs/installation.md#python-venv](docs/installation.md#python-venv).

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
