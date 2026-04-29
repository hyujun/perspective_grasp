# Running the Pipeline

How to launch the perception stack on the host, the ML nodes in Docker, and how to switch between 1 / 2 / 3 camera configurations.

> **Status note.** All five Phase 4 ML nodes (`isaac_foundationpose_tracker`, `sam2_instance_segmentor`, `cosypose_scene_optimizer`, `megapose_ros2_wrapper`, `bundlesdf_unknown_tracker`) ship pluggable real + mock backends, dedicated Docker runtime stages, and multi-camera fan-out. SAM2 is live-verified on hardware; the other four are mock-smoke-tested — they will produce real output once their Docker images are built and weights + meshes are in place. `grasp_pose_planner` ships an antipodal planner + `Hand10DoF` adapter (real 10-DoF preshape mapping still TODO). Everything else (Phase 1–3, fusion, filtering, infra) is live.

## Prerequisites

```bash
cd ${ROS2_WS}/src/perspective_grasp
source .env.live                                      # ROS + workspace + Cyclone DDS
source ${ROS2_WS}/.venv/bin/activate                  # ultralytics & friends
```

All commands below assume the workspace is built and both layers are sourced (`${ROS2_WS}` is **your** colcon workspace root — see [installation.md § Workspace layout](installation.md#workspace-layout)). Build: [build.md](build.md). What `.env.live` sets and why: [installation.md § Host shell env](installation.md#host-shell-env---envlive).

Two failure modes to recognise up front:

- **No `.venv` activated** → `ros2 launch yolo_pcl_cpp_tracker tracker.launch.py` (and any launch that spins it, incl. `perception_system.launch.py`) crashes with `ModuleNotFoundError: No module named 'ultralytics'`. Docker-hosted Phase 4 nodes are unaffected — they use the venv baked inside the container image.
- **No `.env.live` sourced** → host shells default to whatever RMW the system picked, often `rmw_fastrtps_cpp`. Phase 4 containers run Cyclone DDS by design; the mismatch leaves topics listed by `ros2 topic list` but never echoing. See [debugging.md § 4.9](debugging.md#49-phase-4-node-runs-but-host-doesnt-see-its-topics).

## Camera drivers

Launch the camera driver **before** the perception stack, and make its namespace match the `namespace` field in your `camera_config*.yaml`. Phase 1 subscribes on relative topic names (`camera/color/image_raw`, `camera/depth/color/points`, `camera/depth/camera_info`) so they resolve under `perception_system.launch.py`'s `PushRosNamespace`. If the driver publishes on the wrong namespace — or on a different topic layout — subscriptions will never connect.

Rule of thumb: `camera_config_1cam.yaml` (namespace `""`) → driver publishes in root. `camera_config.yaml` (namespaces `/cam0`, `/cam1`, `/cam2`) → one driver per camera, each under its own namespace.

### RealSense (`realsense2_camera`)

The Phase 1 topic convention matches the RealSense driver's native layout — no remapping needed.

**Single camera** (root namespace, pairs with `camera_config_1cam.yaml`):

```bash
ros2 launch realsense2_camera rs_launch.py \
    camera_namespace:=/ \
    camera_name:=camera \
    pointcloud.enable:=true \
    align_depth.enable:=true \
    rgb_camera.color_qos:=SENSOR_DATA \
    depth_module.depth_qos:=SENSOR_DATA \
    depth_module.infra_qos:=SENSOR_DATA
```

`camera_namespace:=/` puts the driver at root (topics `/camera/...`). `ros2 launch` rejects empty argument values, so `camera_namespace:=''` will fail — use `/` for the root namespace.

**Multi-camera** — one driver per camera, namespace must match the YAML:

```bash
# /cam0
ros2 launch realsense2_camera rs_launch.py \
    camera_namespace:=cam0 camera_name:=camera \
    serial_no:='_123456789012' \
    pointcloud.enable:=true align_depth.enable:=true

# /cam1 (and similarly /cam2)
ros2 launch realsense2_camera rs_launch.py \
    camera_namespace:=cam1 camera_name:=camera \
    serial_no:='_223456789012' \
    pointcloud.enable:=true align_depth.enable:=true
```

`serial_no` uses the underscore-prefixed form (`_<SERIAL>`). The `SERIAL_CAMx` placeholders in `camera_config*.yaml` are for documentation — fill in real device serials. For multi-camera hardware sync, set `sync_mode` on the physical connector and keep the YAML's `sync_mode` field consistent (0=independent, 1=master, 2=slave).

### Stereolabs ZED (`zed-ros2-wrapper`)

ZED's native topic names (`rgb/image_rect_color`, `point_cloud/cloud_registered`, `depth/camera_info`) differ from the RealSense-flavored Phase 1 defaults, so you need either topic relays or parameter overrides.

**Launch (per camera)** — ZED's wrapper treats `camera_name` as both the node name and the topic namespace:

```bash
ros2 launch zed_wrapper zed_camera.launch.py \
    camera_model:=zed2i \
    camera_name:=cam0 \
    serial_number:=12345
```

Published topics land under `/cam0/rgb/...`, `/cam0/point_cloud/...`, `/cam0/depth/...`.

**Option A — topic relay** (quickest, keeps Phase 1 defaults untouched):

```bash
ros2 run topic_tools relay /cam0/rgb/image_rect_color         /cam0/camera/color/image_raw
ros2 run topic_tools relay /cam0/point_cloud/cloud_registered /cam0/camera/depth/points
ros2 run topic_tools relay /cam0/depth/camera_info            /cam0/camera/depth/camera_info
```

Repeat per camera namespace.

**Option B — override Phase 1 parameters** to point at ZED's native topic names. Keep them relative so `PushRosNamespace` still prefixes the camera namespace:

```yaml
# Local override of tracker_params.yaml (ZED variant)
yolo_pcl_cpp_tracker:
  ros__parameters:
    detection_topic: yolo/detections                 # unchanged (Phase 1 → Phase 1)
    cloud_topic: point_cloud/cloud_registered        # ZED native
    camera_info_topic: depth/camera_info             # ZED native
```

Also pass `image_topic:=rgb/image_rect_color` to `yolo_byte_tracker`.

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

# Phase 5: Grasp planning action server (antipodal + Hand10DoF)
ros2 launch grasp_pose_planner grasp_planner.launch.py

# Infrastructure
ros2 launch perception_meta_controller meta_controller.launch.py
ros2 launch perception_debug_visualizer debug_visualizer.launch.py
ros2 launch multi_camera_calibration calibration_collect.launch.py
```

## Phase 4: ML nodes (Docker)

Phase 4 services run in GPU containers defined in [docker/docker-compose.yml](../docker/docker-compose.yml). They share `network_mode: host` + `ipc: host`, so topics appear on the same DDS graph as the host nodes.

**Host↔container DDS lockstep:** containers default to Cyclone DDS with the localhost-peers XML — compose hardcodes both `RMW_IMPLEMENTATION` and `CYCLONEDDS_URI` to in-container paths so the host's own `CYCLONEDDS_URI` (a host filesystem path) doesn't leak in and crash Cyclone with "can't open configuration file." Host shells must match: `source <repo>/.env.live` is the supported way to set ROS + workspace + Cyclone env in one shot. Sensor-sized messages (Image, DetectionArray, SegmentationArray) silently drop on RMW mismatch — see [debugging.md § 4.9](debugging.md#49-phase-4-node-runs-but-host-doesnt-see-its-topics) for the diagnostic flow.

```bash
cd ${ROS2_WS}/src/perspective_grasp

# Start individual services
docker compose -f docker/docker-compose.yml up foundationpose -d
docker compose -f docker/docker-compose.yml up sam2           -d
docker compose -f docker/docker-compose.yml up cosypose       -d
docker compose -f docker/docker-compose.yml up megapose       -d
docker compose -f docker/docker-compose.yml up bundlesdf      -d

# Start all
docker compose -f docker/docker-compose.yml up -d

# Follow logs
docker compose -f docker/docker-compose.yml logs -f foundationpose

# Stop all
docker compose -f docker/docker-compose.yml down
```

Model weights mount from service-specific env vars — `$FOUNDATIONPOSE_WEIGHTS`, `$SAM2_WEIGHTS`, `$HAPPYPOSE_WEIGHTS` + `$MEGAPOSE_MESHES` (shared by cosypose + megapose), `$BUNDLESDF_WEIGHTS`. If unset, services fall back to `models/<service>/` at repo root. See [installation.md](installation.md#model-weights).

All five services support multi-camera fan-out via an env var pointing at a `camera_config*.yaml`. The compose file mounts `perception_bringup/config` read-only at `/ws/config`:

```bash
# SAM2: 2-camera fan-out (/cam0 + /cam1)
SAM2_CAMERA_CONFIG=/ws/config/camera_config_2cam.yaml \
  docker compose -f docker/docker-compose.yml up sam2

# FoundationPose: 3-camera fan-out (/cam0 + /cam1 + /cam2)
FOUNDATIONPOSE_CAMERA_CONFIG=/ws/config/camera_config.yaml \
  docker compose -f docker/docker-compose.yml up foundationpose

# CosyPose / MegaPose / BundleSDF use the same contract
COSYPOSE_CAMERA_CONFIG=/ws/config/camera_config_2cam.yaml \
  docker compose -f docker/docker-compose.yml up cosypose
MEGAPOSE_CAMERA_CONFIG=/ws/config/camera_config_2cam.yaml \
  docker compose -f docker/docker-compose.yml up megapose
BUNDLESDF_CAMERA_CONFIG=/ws/config/camera_config_2cam.yaml \
  docker compose -f docker/docker-compose.yml up bundlesdf
```

Under fan-out, CosyPose's action server is also namespaced: each per-camera node exposes `/cam{N}/analyze_scene` rather than a global `/analyze_scene` — clients must pick the right one.

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

## Host profiles & preflight

`perception_system.launch.py` accepts two extra knobs covering the dev↔execution-PC split:

| Arg | Default | Effect |
|-----|---------|--------|
| `host_profile` | `auto` | Picks parameter overrides from [`config/host_profiles/`](../packages/bringup/perception_bringup/config/host_profiles/) (`auto` selects via `nvidia-smi` total VRAM). Valid: `auto` / `dev_8gb` / `prod_16gb` / `cpu_only`. Env override: `PERSPECTIVE_HOST_PROFILE`. |
| `preflight` | `true` | Runs a one-shot driver/torch/CUDA probe at launch and logs a `preflight:` block. Set to `false` (or env `PERSPECTIVE_PREFLIGHT_SKIP=1`) to bypass. |
| `preflight_strict` | `false` | When the probe reports a hard error, abort launch instead of continuing with a warning. |

```bash
# Force the dev profile on a 16 GB box (e.g. while VRAM-budget testing):
ros2 launch perception_bringup perception_system.launch.py host_profile:=dev_8gb

# CI-style headless run with no GPU at all:
PERSPECTIVE_HOST_PROFILE=cpu_only PERSPECTIVE_PREFLIGHT_SKIP=1 \
  ros2 launch perception_bringup perception_system.launch.py
```

Profile YAMLs only override **parameter values** (model size, batch size, mock vs real backend); they never branch the launch graph. The runtime helper `perception_launch_utils.resolve_torch_device()` is the second line of defence — it falls back to CPU even if preflight was skipped or the wrong profile was chosen.

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

# Grasp planning (grasp_pose_planner — antipodal + Hand10DoF)
ros2 action send_goal /plan_grasp perception_msgs/action/PlanGrasp \
  "{target_object_id: 0, grasp_strategy: 'power'}"
```

## Authoring new launch files

All `launch.py` files in this workspace import shared helpers from
[`perception_launch_utils`](../packages/infrastructure/perception_launch_utils/README.md).
Use them instead of hand-rolling `os.path.join(get_package_share_directory(...))` blocks
or copy-pasting `OpaqueFunction + yaml.safe_load + per-camera LifecycleNode` boilerplate.

```python
from launch import LaunchDescription
from launch_ros.actions import Node
from perception_launch_utils import config_path

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='my_pkg',
            executable='my_node',
            parameters=[config_path('my_pkg')],   # <pkg>_params.yaml by default
            output='screen',
        ),
    ])
```

Add `<exec_depend>perception_launch_utils</exec_depend>` to the package's `package.xml`.
For multi-camera fan-out, see `fanout_lifecycle_nodes(...)` in the
[helper README](../packages/infrastructure/perception_launch_utils/README.md).

## Next

- [architecture.md](architecture.md) — topic / TF / QoS reference
- [build.md](build.md) — rebuild after code changes
- [debugging.md](debugging.md) — symptom-driven debugging playbook when something doesn't come up
