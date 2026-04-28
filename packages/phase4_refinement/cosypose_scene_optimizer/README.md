# cosypose_scene_optimizer

ROS 2 hybrid LifecycleNode + Action Server wrapping [CosyPose](https://github.com/agimus-project/happypose) (via happypose) — scene-level multi-object 6D pose optimization, triggered on demand.

## Overview

Caches RGB + detections via subscription callbacks (for freshness), exposes an `analyze_scene` action that runs CosyPose's coarse → refiner → optimization pipeline over the paired snapshot, and publishes the optimized poses as `PoseWithMetaArray` on `/cosypose/optimized_poses` **and** returns them inside the action result.

Concrete inference lives in [`backends/`](cosypose_scene_optimizer/backends/) so the node stays framework-agnostic:

| Backend | Where it runs | Use |
|---|---|---|
| `cosypose` (default) | Docker service `cosypose` (`perspective_grasp/cosypose:latest`) | Real scene-level optimization, GPU |
| `mock` | Anywhere, CPU only | Smoke tests — back-projects bbox centres, returns identity rotations |

## Node

**Node Name**: `cosypose_optimizer` (`LifecycleNode`)

| | |
|---|---|
| **Subscribes** | `image_topic` (`sensor_msgs/Image`), `camera_info_topic` (`sensor_msgs/CameraInfo`), `detections_topic` (`perception_msgs/DetectionArray`) |
| **Publishes** | `poses_topic` (`perception_msgs/PoseWithMetaArray`) |
| **Action Server** | `analyze_scene` (`perception_msgs/action/AnalyzeScene`) |

Under multi-camera fan-out each instance exposes `/cam{N}/analyze_scene` — clients must pick the right namespace.

### `AnalyzeScene` action

- **Goal**: `roi` polygon (optional), `target_classes` filter (optional)
- **Feedback**: progress 0.0 → 1.0, stage label (`pair` → `coarse` → `publish`)
- **Result**: `optimized_poses`, `relations` (**currently returned empty** — pose relation inference is a follow-up), `success`, `message`

## Parameters

| Parameter | Default | Description |
|---|---|---|
| `backend` | `"cosypose"` | Backend key (`cosypose` \| `mock`) |
| `image_topic` | `/camera/color/image_raw` | RGB input |
| `camera_info_topic` | `/camera/color/camera_info` | Intrinsics source |
| `detections_topic` | `/yolo/detections` | YOLO box + class prompts |
| `poses_topic` | `/cosypose/optimized_poses` | Output stream (mirrors the action result) |
| `action_name` | `analyze_scene` | Action server name |
| `sync_tolerance_sec` | `0.5` | Max \|t_frame − t_detections\| for goal-time pairing |
| `mesh_dir` | `/ws/models/megapose/meshes` | Flat dir: one `<class>.(obj\|ply)` per YOLO class (shared with MegaPose) |
| `mesh_units` | `"m"` | `m` \| `mm` |
| `coarse_run_id` | `coarse-bop-ycbv-synt+real` | happypose coarse checkpoint id |
| `refiner_run_id` | `refiner-bop-ycbv-synt+real` | happypose refiner checkpoint id |
| `n_coarse_iterations` | `1` | Coarse iterations |
| `n_refiner_iterations` | `4` | Refiner iterations |
| `score_threshold` | `0.3` | Drops sub-threshold pose outputs |
| `device` | `"auto"` | Torch device — `auto`/`cuda`/`cuda:N`/`cpu`. See [perception_launch_utils.resolve_torch_device](../../infrastructure/perception_launch_utils/README.md#resolve_torch_devicerequested-logger---deviceresolution); falls back to cpu with a WARN if CUDA is unusable. |

## Mesh registry

Shared with MegaPose: `$MEGAPOSE_MESHES` → `/ws/models/megapose/meshes`, one `<class>.(obj|ply)` per YOLO class. `.stl` is not supported (happypose's Panda3dBatchRenderer only takes `.obj|.ply`).

## Running

### Host (mock backend, CPU-only)

```bash
colcon build --packages-select cosypose_scene_optimizer
source install/setup.bash
ros2 run cosypose_scene_optimizer cosypose_node --ros-args -p backend:=mock
# in another shell:
ros2 lifecycle set /cosypose_optimizer configure
ros2 lifecycle set /cosypose_optimizer activate

# Trigger scene analysis
ros2 action send_goal /analyze_scene perception_msgs/action/AnalyzeScene \
  "{target_classes: [], roi: []}" --feedback
```

### Docker (real CosyPose backend)

```bash
# One-time: build the cosypose-runtime image (~15 min; shared with megapose).
docker compose -f docker/docker-compose.yml build cosypose

# Place weights + meshes (shared mesh layout with MegaPose):
#   $HAPPYPOSE_WEIGHTS/experiments/<coarse-run>/
#   $HAPPYPOSE_WEIGHTS/experiments/<refiner-run>/
#   $MEGAPOSE_MESHES/<class>.(obj|ply)
export HAPPYPOSE_WEIGHTS=/path/to/models/happypose
export MEGAPOSE_MESHES=/path/to/models/megapose/meshes

# Single-cam:
docker compose -f docker/docker-compose.yml up cosypose

# Multi-cam fan-out (/cam0 + /cam1, each with /cam{N}/analyze_scene):
COSYPOSE_CAMERA_CONFIG=/ws/config/camera_config_2cam.yaml \
  docker compose -f docker/docker-compose.yml up cosypose
```

> Detailed weights/meshes layout + happypose CLI download recipe: [docs/model_assets.md](../../../docs/model_assets.md).

### Multi-camera fan-out

Pass `camera_config:=<yaml>` to the launch file to spawn one LifecycleNode per camera declared in `perception_system.cameras`. Each instance gets its own namespace (`/cam0/`, `/cam1/`, …) with all its topics **and** its action server prefixed. See [CLAUDE.md](../../../CLAUDE.md).

### Host profile overrides

`host_profile:=<dev_8gb|prod_16gb|cpu_only|auto>` (default `auto`, env `PERSPECTIVE_HOST_PROFILE`) selects parameter overrides keyed by node name `cosypose_optimizer`. Profile YAMLs live at [`packages/bringup/perception_bringup/config/host_profiles/`](../../bringup/perception_bringup/config/host_profiles/); `cpu_only` flips this node's `backend` to `mock`. Overrides are appended last in `parameters=[...]` so they win on conflict.

## Dependencies

- **ROS**: `rclpy`, `sensor_msgs`, `geometry_msgs`, `perception_msgs`, `cv_bridge`
- **Python (host)**: numpy, PyYAML
- **Python (cosypose backend, Docker only)**: torch 2.6, torchvision 0.21, PyTorch3D v0.7.9, happypose (clone at `/opt/happypose`, cosypose subproject installed `-e` for its pybind11 C++ ext), panda3d, pin (pinocchio), pybullet. Provisioned by `docker/Dockerfile` stage `cosypose-runtime` (shared with MegaPose). See [`memory/happypose_install_trap.md`](../../../.claude/projects/.../memory/happypose_install_trap.md) for the install recipe.
