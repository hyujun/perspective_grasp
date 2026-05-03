# bundlesdf_unknown_tracker

ROS 2 LifecycleNode wrapping [NVlabs/BundleSDF](https://github.com/NVlabs/BundleSDF) — 6D tracking of **unknown** objects (no CAD / mesh prior). Combines RGB-D frames with SAM2 masks to jointly build a neural SDF and pose graph per track.

## Overview

Pairs RGB + depth frames with SAM2 instance masks (by timestamp) and runs BundleSDF's per-track BundleTrack + neural SDF pipeline to produce a 6D pose per object. Output is published as `PoseWithMetaArray` on `/bundlesdf/raw_poses` (or `/{cam_ns}/bundlesdf/raw_poses` under multi-camera fan-out).

Concrete inference lives in [`backends/`](bundlesdf_unknown_tracker/backends/) so the node stays framework-agnostic:

| Backend | Where it runs | Use |
|---|---|---|
| `bundlesdf` (default) | Docker service `bundlesdf` (`perspective_grasp/bundlesdf:latest`) | Real mesh-free 6D tracking, GPU |
| `mock` | Anywhere, CPU only | Smoke tests — depth-unprojects each mask centroid, returns identity rotation |

BundleSDF consumes SAM2 masks (not YOLO boxes), so the `sam2` Docker service must be running concurrently when using the `bundlesdf` backend.

## Node

**Node Name**: `bundlesdf_tracker` (`LifecycleNode`)

| | |
|---|---|
| **Subscribes** | `image_topic` (`sensor_msgs/Image`), `depth_topic` (`sensor_msgs/Image`), `camera_info_topic` (`sensor_msgs/CameraInfo`), `masks_topic` (`perception_msgs/SegmentationArray`) |
| **Publishes** | `poses_topic` (`perception_msgs/PoseWithMetaArray`) |

All topic names are parameters — see [`config/bundlesdf_params.yaml`](config/bundlesdf_params.yaml).

## Parameters

| Parameter | Default | Description |
|---|---|---|
| `backend` | `"bundlesdf"` | Backend key (`bundlesdf` \| `mock`) |
| `image_topic` | `/camera/color/image_raw` | RGB input |
| `depth_topic` | `/camera/depth/image_rect_raw` | Depth input (16UC1 mm or 32FC1 m) |
| `camera_info_topic` | `/camera/color/camera_info` | Intrinsics source |
| `masks_topic` | `/sam2/masks` | SAM2 instance masks |
| `poses_topic` | `/bundlesdf/raw_poses` | Output |
| `frame_queue_size` | `10` | Max RGB+depth frames buffered for mask pairing |
| `sync_tolerance_sec` | `0.05` | Max \|t_frame − t_masks\| |
| `stale_track_frames` | `30` | Drop backend per-track state after N mask frames without that id |
| `min_mask_confidence` | `0.0` | Filter SAM2 masks below this |
| `min_mask_area` | `400` | Skip masks smaller than this many pixels |
| `bundlesdf_src` | `/opt/BundleSDF` | Added to `sys.path` before importing `bundlesdf` (inside container) |
| `cfg_track_path` | `/opt/BundleSDF/BundleTrack/config_ho3d.yml` | BundleTrack config (override to mount tuned configs) |
| `cfg_nerf_path` | `/opt/BundleSDF/nerf_runner.yml` | NeRF runner config |
| `out_dir` | `/ws/models/bundlesdf/out` | Per-track SDF / debug output root (must be writable) |
| `shorter_side` | `480` | BundleSDF input image short-side resize |
| `depth_max_m` | `2.0` | Clip invalid depth pixels |
| `device` | `"auto"` | Torch device — `auto`/`cuda`/`cuda:N`/`cpu`. See [perception_launch_utils.resolve_torch_device](../../infrastructure/perception_launch_utils/README.md#resolve_torch_devicerequested-logger---deviceresolution); falls back to cpu with a WARN if CUDA is unusable. |

## Working directory

Unlike the other Phase 4 nodes, BundleSDF is mesh-free — no mesh registry. Instead it writes per-track SDF + debug output to `out_dir` (default `/ws/models/bundlesdf/out`, mounted from `$BUNDLESDF_WEIGHTS/out` on the host). The directory must exist and be writable by the container UID.

## Running

### Host (mock backend, CPU-only)

```bash
colcon build --packages-select bundlesdf_unknown_tracker
source install/setup.bash
ros2 launch bundlesdf_unknown_tracker bundlesdf.launch.py --ros-args -p backend:=mock
# in another shell:
ros2 lifecycle set /bundlesdf_tracker configure
ros2 lifecycle set /bundlesdf_tracker activate
```

### Docker (real BundleSDF backend)

```bash
# One-time: build the bundlesdf-runtime image (CUDA 12.6 + kaolin + nvdiffrast + PyTorch3D, ~15 min).
docker compose -f docker/docker-compose.yml build bundlesdf

# Place a writable out/ dir (mesh-free — no weights required):
#   $BUNDLESDF_WEIGHTS/out/
export BUNDLESDF_WEIGHTS=/path/to/models/bundlesdf

# BundleSDF consumes SAM2 masks — start sam2 + bundlesdf together:
docker compose -f docker/docker-compose.yml up sam2 bundlesdf

# Multi-cam fan-out (/cam0 + /cam1):
BUNDLESDF_CAMERA_CONFIG=/ws/config/camera_config_2cam.yaml \
  docker compose -f docker/docker-compose.yml up sam2 bundlesdf
```

> Detailed directory layout + optional LoFTR/XMem weights: [docs/model_assets.md](../../../docs/model_assets.md).

### Multi-camera fan-out

Pass `camera_config:=<yaml>` to the launch file to spawn one LifecycleNode per camera declared in `perception_system.cameras`. Each instance gets its own namespace (`/cam0/`, `/cam1/`, …) with all its topics prefixed. See [CLAUDE.md](../../../CLAUDE.md) for the multi-camera convention.

### Host profile overrides

`host_profile:=<dev_8gb|prod_16gb|cpu_only|auto>` (default `auto`, env `PERSPECTIVE_HOST_PROFILE`) selects parameter overrides keyed by node name `bundlesdf_tracker`. The `dev_8gb` profile shrinks `shorter_side` to 320 to fit 8 GB-class VRAM; `cpu_only` flips `backend` to `mock`. See [`host_profiles/`](../../infrastructure/perception_launch_utils/host_profiles/) (shipped with `perception_launch_utils`).

## Dependencies

- **ROS**: `rclpy`, `sensor_msgs`, `geometry_msgs`, `perception_msgs`, `cv_bridge`
- **Python (host)**: numpy, PyYAML — available from `python3-numpy` / `python3-yaml`
- **Python (bundlesdf backend, Docker only)**: torch 2.6, torchvision 0.21, kaolin 0.17, nvdiffrast 0.3.3, PyTorch3D v0.7.9, Open3D 0.18, trimesh, pyrender, kornia, NVlabs/BundleSDF (clone at `/opt/BundleSDF`). Provisioned by `docker/Dockerfile` stage `bundlesdf-runtime`.
