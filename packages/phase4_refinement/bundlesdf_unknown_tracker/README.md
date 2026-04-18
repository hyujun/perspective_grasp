# bundlesdf_unknown_tracker

ROS 2 LifecycleNode wrapping [NVlabs/BundleSDF](https://github.com/NVlabs/BundleSDF) for 6D tracking of **unknown** objects (no CAD / mesh prior required). Combines RGB-D frames with SAM2 masks to jointly build a neural SDF and pose graph per track.

Shares the pluggable-backend shape used by `sam2_instance_segmentor` / `isaac_foundationpose_tracker` / `cosypose_scene_optimizer` / `megapose_ros2_wrapper` — real inference runs in the `bundlesdf` Docker service; a mock backend is available for CPU-only smoke tests.

## Node

**Node Name**: `bundlesdf_tracker` (LifecycleNode)

| | |
|---|---|
| **Subscribes** | `/camera/color/image_raw` (`Image`), `/camera/depth/image_rect_raw` (`Image`), `/camera/color/camera_info` (`CameraInfo`), `/sam2/masks` (`SegmentationArray`) |
| **Publishes** | `/bundlesdf/raw_poses` (`PoseWithMetaArray`) |

Under multi-camera fan-out topics are prefixed with `/{cam_ns}/…` — see `launch/bundlesdf.launch.py`.

## Backends

| Key | Runs on | Notes |
|---|---|---|
| `bundlesdf` | GPU (Docker `bundlesdf` service) | Real NVlabs BundleSDF. Expects `/opt/BundleSDF` on `PYTHONPATH` and CUDA 12.6 + torch 2.6 + kaolin 0.17 + nvdiffrast 0.3.3 + PyTorch3D v0.7.9 (all provisioned by `docker/Dockerfile` `bundlesdf-runtime` stage). |
| `mock` | CPU | Depth-unprojected mask centroid with identity rotation. No weights. Used for smoke tests. |

Switch with the `backend` parameter (see `config/bundlesdf_params.yaml`).

## Parameters

Defaults in `config/bundlesdf_params.yaml` — the `/**:` wildcard makes the same file apply to both single-cam (`/bundlesdf_tracker`) and fan-out (`/cam{N}/bundlesdf_tracker`) instances.

| Parameter | Default | Description |
|-----------|---------|-------------|
| `backend` | `bundlesdf` | `bundlesdf` \| `mock` |
| `image_topic` / `depth_topic` / `camera_info_topic` / `masks_topic` / `poses_topic` | see YAML | Topic remaps. |
| `frame_queue_size` | `10` | Max RGB+depth frames buffered for mask pairing. |
| `sync_tolerance_sec` | `0.05` | Max \|t_frame − t_masks\|. |
| `stale_track_frames` | `30` | Drop backend per-track state after N mask frames without that id. |
| `min_mask_confidence` | `0.0` | Filter SAM2 masks below this. |
| `bundlesdf_src` | `/opt/BundleSDF` | Added to `sys.path` before importing `bundlesdf`. |
| `cfg_track_path` / `cfg_nerf_path` | repo defaults | BundleSDF's bundled YAMLs; override to mount tuned configs. |
| `out_dir` | `/ws/models/bundlesdf/out` | Per-track SDF / debug output root. |
| `shorter_side` | `480` | BundleSDF input image short-side. |
| `depth_max_m` | `2.0` | Clip invalid depth pixels. |
| `min_mask_area` | `400` | Skip masks smaller than this. |
| `device` | `cuda` | Inference device. |

## Run (mock, single camera)

```bash
colcon build --packages-select bundlesdf_unknown_tracker
source install/setup.bash
ros2 launch bundlesdf_unknown_tracker bundlesdf.launch.py \
  --ros-args -p backend:=mock
ros2 lifecycle set /bundlesdf_tracker configure
ros2 lifecycle set /bundlesdf_tracker activate
```

## Run (real, Docker)

```bash
docker compose -f docker/docker-compose.yml build bundlesdf   # ~15 min first time
export BUNDLESDF_WEIGHTS=/path/to/models/bundlesdf             # optional
export BUNDLESDF_CAMERA_CONFIG=/ws/config/camera_config_2cam.yaml  # optional, fan-out
docker compose -f docker/docker-compose.yml up bundlesdf
```

## Dependencies

- `rclpy`, `sensor_msgs`, `geometry_msgs`, `perception_msgs`, `cv_bridge`, `python3-numpy`, `python3-yaml`
- GPU backend (Docker): torch 2.6.0 / torchvision 0.21.0 (cu126), kaolin 0.17.0, nvdiffrast 0.3.3, PyTorch3D v0.7.9, Open3D 0.18, trimesh, pyrender, kornia, NVlabs/BundleSDF
