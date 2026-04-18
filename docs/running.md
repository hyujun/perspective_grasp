# Running the Pipeline

How to launch the perception stack on the host, the ML nodes in Docker, and how to switch between 1 / 2 / 3 camera configurations.

> **Status note.** `isaac_foundationpose_tracker` and `sam2_instance_segmentor` are live (real + mock backends, Docker runtime stages, multi-cam fan-out). `grasp_pose_planner` ships an antipodal planner + `Hand10DoF` adapter. `megapose_ros2_wrapper`, `cosypose_scene_optimizer`, and `bundlesdf_unknown_tracker` remain stubs — their launches start cleanly but produce no output until implemented. Everything else (Phase 1–3, fusion, filtering, infra) is live.

## Prerequisites

```bash
cd ~/ros2_ws/perspective_ws
source install/setup.bash
```

All commands below assume the workspace is built and sourced. Build instructions: [build.md](build.md).

## Quick start — single camera

Launches the full host-side pipeline (YOLO tracker + ICP + pose filter + fusion + debug visualizer) with a single connected camera:

```bash
ros2 launch perception_bringup perception_system.launch.py
```

## Launch by phase

Use these when iterating on a single phase.

```bash
# Phase 1: Detection & pose estimation (per-camera)
ros2 launch yolo_pcl_cpp_tracker tracker.launch.py

# Phase 1 (multi-camera): generated per-camera tracker fleet
ros2 launch perception_bringup phase1_bringup.launch.py

# Phase 2: Multi-camera fusion
ros2 launch cross_camera_associator associator.launch.py
ros2 launch pcl_merge_node merge.launch.py

# Phase 3: Filtering & smoothing
ros2 launch pose_filter_cpp pose_filter.launch.py
ros2 launch pose_graph_smoother smoother.launch.py

# Phase 5: Grasp planning action server (stub)
ros2 launch grasp_pose_planner grasp_planner.launch.py

# Infrastructure
ros2 launch perception_meta_controller meta_controller.launch.py
ros2 launch perception_debug_visualizer debug_visualizer.launch.py
ros2 launch multi_camera_calibration calibration_collect.launch.py
```

## Phase 4: ML nodes (Docker)

Phase 4 services run in GPU containers defined in [docker/docker-compose.yml](../docker/docker-compose.yml). They share `network_mode: host` + `ipc: host`, so topics appear on the same DDS graph as the host nodes.

```bash
cd ~/ros2_ws/perspective_ws/src/perspective_grasp

# Start individual services
docker compose -f docker/docker-compose.yml up foundationpose -d
docker compose -f docker/docker-compose.yml up sam2          -d
docker compose -f docker/docker-compose.yml up cosypose      -d
docker compose -f docker/docker-compose.yml up bundlesdf     -d

# Start all
docker compose -f docker/docker-compose.yml up -d

# Follow logs
docker compose -f docker/docker-compose.yml logs -f foundationpose

# Stop all
docker compose -f docker/docker-compose.yml down
```

Model weights mount from `$FOUNDATIONPOSE_WEIGHTS` / `$SAM2_WEIGHTS` / `$COSYPOSE_WEIGHTS` / `$BUNDLESDF_WEIGHTS` (or `models/<service>/` at repo root). See [installation.md](installation.md#model-weights).

Both SAM2 and FoundationPose support multi-camera fan-out via an env var that points at a `camera_config*.yaml`. The compose file mounts the `perception_bringup/config` directory read-only at `/ws/config`:

```bash
# SAM2: 2-camera fan-out (/cam0 + /cam1)
SAM2_CAMERA_CONFIG=/ws/config/camera_config_2cam.yaml \
  docker compose -f docker/docker-compose.yml up sam2

# FoundationPose: 3-camera fan-out (/cam0 + /cam1 + /cam2)
FOUNDATIONPOSE_CAMERA_CONFIG=/ws/config/camera_config.yaml \
  docker compose -f docker/docker-compose.yml up foundationpose
```

## Camera configuration

The bringup is config-driven — pick the YAML that matches your physical setup. Configs live in the `perception_bringup` share directory after build.

```bash
BRINGUP_CFG=$(ros2 pkg prefix perception_bringup)/share/perception_bringup/config

# 1 camera
ros2 launch perception_bringup perception_system.launch.py \
    camera_config:=$BRINGUP_CFG/camera_config_1cam.yaml

# 2 cameras
ros2 launch perception_bringup perception_system.launch.py \
    camera_config:=$BRINGUP_CFG/camera_config_2cam.yaml

# 3 cameras (default)
ros2 launch perception_bringup perception_system.launch.py \
    camera_config:=$BRINGUP_CFG/camera_config.yaml
```

> The launch path is unified: `N=1` is treated as "N=1 multi-camera", not as a special case. Per-camera nodes are namespaced `/cam0/`, `/cam1/`, `/cam2/`. Do not add `if N==1` branches — change the config instead.

## Pipeline modes

`perception_meta_controller` exposes a `SetMode` service that toggles which downstream nodes are active.

| Mode | Active nodes |
|------|--------------|
| `NORMAL` | YOLO tracker + ICP + pose filter |
| `HIGH_PRECISION` | `NORMAL` + FoundationPose + pose graph smoother |
| `SCENE_ANALYSIS` | YOLO tracker + SAM2 + CosyPose + pose filter |

```bash
ros2 service call /meta_controller/set_mode perception_msgs/srv/SetMode "{mode: 'HIGH_PRECISION'}"
```

## Verification

```bash
# Nodes & topics
ros2 node list
ros2 topic list

# Per-camera rates
ros2 topic hz /cam0/yolo_tracker/raw_poses
ros2 topic hz /pose_filter/filtered_poses

# TF graph snapshot
ros2 run tf2_tools view_frames

# Filtered pose stream
ros2 topic echo /pose_filter/filtered_poses

# Pipeline status
ros2 topic echo /meta_controller/active_pipeline
```

## Action servers

Heavy operations are exposed as actions rather than services, so the control loop never blocks on them.

```bash
# Scene analysis (perception_meta_controller)
ros2 action send_goal /meta_controller/analyze_scene perception_msgs/action/AnalyzeScene "{...}"

# Grasp planning (grasp_pose_planner — stub)
ros2 action send_goal /grasp_planner/plan_grasp perception_msgs/action/PlanGrasp "{...}"
```

## Next

- [architecture.md](architecture.md) — topic / TF / QoS reference
- [build.md](build.md) — rebuild after code changes
