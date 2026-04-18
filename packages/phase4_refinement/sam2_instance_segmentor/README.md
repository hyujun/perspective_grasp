# sam2_instance_segmentor

ROS 2 LifecycleNode wrapping [Meta SAM2](https://github.com/facebookresearch/sam2) — instance segmentation from YOLO box prompts.

## Overview

Pairs RGB frames with YOLO detections (by timestamp) and runs SAM2 to produce per-instance binary masks. Output is published as `SegmentationArray` on `/sam2/masks` (or `/{cam_ns}/sam2/masks` under multi-camera fan-out).

Concrete inference lives in [`backends/`](sam2_instance_segmentor/backends/) so the node stays framework-agnostic:

| Backend | Where it runs | Use |
|---|---|---|
| `sam2` (default) | Docker service `sam2` (`perspective_grasp/sam2:latest`) | Real segmentation, GPU |
| `mock` | Anywhere, CPU only | Smoke tests — paints an elliptical mask per bbox |

## Node

**Node Name**: `sam2_segmentor` (`LifecycleNode`)

| | |
|---|---|
| **Subscribes** | `image_topic` (`sensor_msgs/Image`), `detections_topic` (`perception_msgs/DetectionArray`) |
| **Publishes** | `masks_topic` (`perception_msgs/SegmentationArray`) |

All topic names are parameters — see [`config/sam2_params.yaml`](config/sam2_params.yaml).

## Parameters

| Parameter | Default | Description |
|---|---|---|
| `backend` | `"sam2"` | Backend key (`sam2` \| `mock`) |
| `image_topic` | `/camera/color/image_raw` | RGB input |
| `detections_topic` | `/yolo/detections` | YOLO box prompts |
| `masks_topic` | `/sam2/masks` | Output |
| `image_queue_size` | `10` | Max RGB frames kept for pairing |
| `sync_tolerance_sec` | `0.05` | Max \|t_image − t_detections\| |
| `min_detection_confidence` | `0.0` | Drops sub-threshold YOLO boxes (re-read each frame — live-tunable) |
| `model_checkpoint` | `/ws/models/sam2/sam2_hiera_large.pt` | SAM2 checkpoint path (inside container) |
| `model_config` | `sam2_hiera_l.yaml` | Hydra config name |
| `pred_iou_thresh` | `0.88` | Mask-quality threshold |
| `multimask_output` | `false` | If true, return the best of 3 multi-hypothesis masks |
| `device` | `"cuda"` | Torch device |

## Running

### Host (mock backend, CPU-only)

```bash
colcon build --packages-select sam2_instance_segmentor
source install/setup.bash
ros2 run sam2_instance_segmentor sam2_segmentor_node --ros-args -p backend:=mock
# in another shell:
ros2 lifecycle set /sam2_segmentor configure
ros2 lifecycle set /sam2_segmentor activate
```

### Docker (real SAM2 backend)

```bash
# One-time: build the sam2-runtime image (~8 min).
docker compose -f docker/docker-compose.yml build sam2

# Place the SAM2 checkpoint:
#   $SAM2_WEIGHTS/sam2_hiera_large.pt
export SAM2_WEIGHTS=/path/to/models/sam2

# Single-cam:
docker compose -f docker/docker-compose.yml up sam2

# Multi-cam fan-out (/cam0 + /cam1):
SAM2_CAMERA_CONFIG=/ws/config/camera_config_2cam.yaml \
  docker compose -f docker/docker-compose.yml up sam2
```

### Multi-camera fan-out

Pass `camera_config:=<yaml>` to the launch file to spawn one LifecycleNode per camera declared in `perception_system.cameras`. Each instance gets its own namespace (`/cam0/`, `/cam1/`, …) with all its topics prefixed. See [CLAUDE.md](../../../CLAUDE.md) for the multi-camera convention.

## Dependencies

- **ROS**: `rclpy`, `sensor_msgs`, `perception_msgs`, `cv_bridge`
- **Python (host)**: numpy, PyYAML — available from `python3-numpy` / `python3-yaml`
- **Python (sam2 backend, Docker only)**: torch 2.6, torchvision 0.21, Meta SAM2. Installed by `docker/Dockerfile` stage `sam2-runtime`.
