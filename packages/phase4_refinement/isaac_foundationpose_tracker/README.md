# isaac_foundationpose_tracker

ROS 2 LifecycleNode wrapping [NVIDIA FoundationPose](https://github.com/NVlabs/FoundationPose) — zero-shot 6D pose estimation from RGB-D + per-class object meshes.

## Overview

Pairs aligned RGB + depth frames with YOLO detections (by timestamp), looks up the matching mesh per detection class, and runs FoundationPose to produce a 6D pose per object. Output is published as `PoseWithMetaArray` on `/foundationpose/raw_poses` (or `/{cam_ns}/foundationpose/raw_poses` under multi-camera fan-out).

Concrete inference lives in [`backends/`](isaac_foundationpose_tracker/backends/) so the node stays framework-agnostic:

| Backend | Where it runs | Use |
|---|---|---|
| `foundationpose` (default) | Docker service `foundationpose` (`perspective_grasp/foundationpose:latest`) | Real 6D tracking, GPU |
| `mock` | Anywhere, CPU only | Smoke tests — depth-unprojects each bbox centre, returns identity rotation |

## Node

**Node Name**: `foundationpose_tracker` (`LifecycleNode`)

| | |
|---|---|
| **Subscribes** | `image_topic` (`sensor_msgs/Image`), `depth_topic` (`sensor_msgs/Image`), `camera_info_topic` (`sensor_msgs/CameraInfo`), `detections_topic` (`perception_msgs/DetectionArray`) |
| **Publishes** | `poses_topic` (`perception_msgs/PoseWithMetaArray`) |

All topic names are parameters — see [`config/foundationpose_params.yaml`](config/foundationpose_params.yaml).

## Parameters

| Parameter | Default | Description |
|---|---|---|
| `backend` | `"foundationpose"` | Backend key (`foundationpose` \| `mock`) |
| `image_topic` | `/camera/color/image_raw` | RGB input |
| `depth_topic` | `/camera/depth/image_rect_raw` | Depth input (16UC1 mm or 32FC1 m) |
| `camera_info_topic` | `/camera/color/camera_info` | Intrinsics source |
| `detections_topic` | `/yolo/detections` | YOLO box + class prompts |
| `poses_topic` | `/foundationpose/raw_poses` | Output |
| `frame_queue_size` | `10` | Max RGB+depth frames kept for pairing |
| `sync_tolerance_sec` | `0.05` | Max \|t_frame − t_detections\| |
| `min_detection_confidence` | `0.0` | Drops sub-threshold YOLO boxes (re-read each frame — live-tunable) |
| `mesh_dir` | `/ws/models/foundationpose/meshes` | Dir with one `<class_name>.(obj\|ply\|stl)` per YOLO class |
| `refine_iterations` | `5` | FoundationPose refiner iterations |
| `score_threshold` | `0.5` | Drops sub-threshold pose outputs |
| `depth_max_m` | `2.0` | Zero-out depth beyond this range |
| `device` | `"auto"` | Torch device — `auto`/`cuda`/`cuda:N`/`cpu`. See [perception_launch_utils.resolve_torch_device](../../infrastructure/perception_launch_utils/README.md#resolve_torch_devicerequested-logger---deviceresolution); falls back to cpu with a WARN if CUDA is unusable. |

## Mesh registry

`mesh_dir` is flat: one file per YOLO class, filename stem = class name (case-insensitive). A prompt whose class has no matching mesh is silently skipped so the pipeline can run with partial coverage.

```
models/foundationpose/meshes/
├── box.obj
├── cup.ply
└── mug.stl
```

## Running

### Host (mock backend, CPU-only)

```bash
colcon build --packages-select isaac_foundationpose_tracker
source install/setup.bash
ros2 run isaac_foundationpose_tracker foundationpose_node --ros-args -p backend:=mock
# in another shell:
ros2 lifecycle set /foundationpose_tracker configure
ros2 lifecycle set /foundationpose_tracker activate
```

### Docker (real FoundationPose backend)

```bash
# One-time: build the foundationpose-runtime image (CUDA 12.6 + kaolin + nvdiffrast, ~15 min).
docker compose -f docker/docker-compose.yml build foundationpose

# Place meshes + weights:
#   $FOUNDATIONPOSE_WEIGHTS/meshes/*.obj
#   $FOUNDATIONPOSE_WEIGHTS/weights/   (FoundationPose refiner/scorer checkpoints)
export FOUNDATIONPOSE_WEIGHTS=/path/to/models/foundationpose

# Single-cam:
docker compose -f docker/docker-compose.yml up foundationpose

# Multi-cam fan-out (/cam0 + /cam1):
FOUNDATIONPOSE_CAMERA_CONFIG=/ws/config/camera_config_2cam.yaml \
  docker compose -f docker/docker-compose.yml up foundationpose
```

> Detailed weights/meshes layout + Google Drive download recipe: [docs/model_assets.md](../../../docs/model_assets.md).

### Multi-camera fan-out

Pass `camera_config:=<yaml>` to the launch file to spawn one LifecycleNode per camera declared in `perception_system.cameras`. Each instance gets its own namespace (`/cam0/`, `/cam1/`, …) with all its topics prefixed. This mirrors the pattern in `sam2_segmentor.launch.py` — see [CLAUDE.md](../../../CLAUDE.md) for the multi-camera convention.

### Host profile overrides

`host_profile:=<dev_8gb|prod_16gb|cpu_only|auto>` (default `auto`, env `PERSPECTIVE_HOST_PROFILE`) selects parameter overrides keyed by node name `foundationpose_tracker`. The `dev_8gb` profile drops `refine_iterations` to 3 to fit 8 GB-class VRAM; `cpu_only` flips `backend` to `mock`. See [`config/host_profiles/`](../../bringup/perception_bringup/config/host_profiles/).

## Dependencies

- **ROS**: `rclpy`, `sensor_msgs`, `geometry_msgs`, `perception_msgs`, `cv_bridge`
- **Python (host)**: numpy, PyYAML — available from `python3-numpy` / `python3-yaml`
- **Python (foundationpose backend, Docker only)**: torch 2.6, torchvision 0.21, kaolin 0.17, nvdiffrast 0.3.3, trimesh, pyrender, open3d. Installed by `docker/Dockerfile` stage `foundationpose-runtime`.
