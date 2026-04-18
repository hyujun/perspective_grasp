# megapose_ros2_wrapper

ROS 2 LifecycleNode wrapping [MegaPose](https://github.com/agimus-project/happypose) (via happypose) — RGB-only zero-shot 6D pose estimation from a CAD / mesh prior.

## Overview

Pairs RGB frames with YOLO detections (by timestamp), looks up the matching mesh per detection class, and runs MegaPose's coarse → refiner pipeline to produce a 6D pose per object. Output is published as `PoseWithMetaArray` on `/megapose/raw_poses` (or `/{cam_ns}/megapose/raw_poses` under multi-camera fan-out).

Concrete inference lives in [`backends/`](megapose_ros2_wrapper/backends/) so the node stays framework-agnostic:

| Backend | Where it runs | Use |
|---|---|---|
| `megapose` (default) | Docker service `megapose` (shares `perspective_grasp/cosypose:latest`) | Real RGB-only 6D pose, GPU |
| `mock` | Anywhere, CPU only | Smoke tests — back-projects bbox centre through a fake depth |

MegaPose is RGB-only by design; depth is not required (and not consumed). The `cosypose-runtime` Docker stage is shared with `cosypose_scene_optimizer` because happypose bundles both.

## Node

**Node Name**: `megapose_tracker` (`LifecycleNode`)

| | |
|---|---|
| **Subscribes** | `image_topic` (`sensor_msgs/Image`), `camera_info_topic` (`sensor_msgs/CameraInfo`), `detections_topic` (`perception_msgs/DetectionArray`) |
| **Publishes** | `poses_topic` (`perception_msgs/PoseWithMetaArray`) |

All topic names are parameters — see [`config/megapose_params.yaml`](config/megapose_params.yaml).

## Parameters

| Parameter | Default | Description |
|---|---|---|
| `backend` | `"megapose"` | Backend key (`megapose` \| `mock`) |
| `image_topic` | `/camera/color/image_raw` | RGB input |
| `camera_info_topic` | `/camera/color/camera_info` | Intrinsics source |
| `detections_topic` | `/yolo/detections` | YOLO box + class prompts |
| `poses_topic` | `/megapose/raw_poses` | Output |
| `frame_queue_size` | `10` | Max RGB frames kept for pairing |
| `sync_tolerance_sec` | `0.05` | Max \|t_frame − t_detections\| |
| `min_detection_confidence` | `0.0` | Drops sub-threshold YOLO boxes |
| `mesh_dir` | `/ws/models/megapose/meshes` | Flat dir: one `<class_name>.(obj\|ply)` per YOLO class |
| `mesh_units` | `"m"` | `m` \| `mm` — scale hint for `happypose.RigidObject` |
| `model_name` | `"megapose-1.0-RGB-multi-hypothesis"` | Key into `happypose.toolbox.utils.load_model.NAMED_MODELS` |
| `score_threshold` | `0.5` | Drops sub-threshold pose outputs |
| `device` | `"cuda"` | Torch device |

Inference parameters (refiner iterations, hypothesis count) come from `NAMED_MODELS[model_name]["inference_parameters"]`, **not** from yaml. To tune them, switch to a different `model_name` or patch the backend's `self._inference_params` override map.

## Mesh registry

`mesh_dir` is flat: one file per YOLO class, filename stem = class name (case-insensitive). Only `.obj` and `.ply` are supported — happypose's `Panda3dBatchRenderer` does not accept `.stl`.

```
models/megapose/meshes/
├── box.obj
├── cup.ply
└── mug.obj
```

A prompt whose class has no matching mesh is silently skipped.

## Running

### Host (mock backend, CPU-only)

```bash
colcon build --packages-select megapose_ros2_wrapper
source install/setup.bash
ros2 run megapose_ros2_wrapper megapose_node --ros-args -p backend:=mock
# in another shell:
ros2 lifecycle set /megapose_tracker configure
ros2 lifecycle set /megapose_tracker activate
```

### Docker (real MegaPose backend)

```bash
# One-time: build the cosypose-runtime image (shared with cosypose; ~15 min).
docker compose -f docker/docker-compose.yml build megapose

# Place weights + meshes:
#   $HAPPYPOSE_WEIGHTS/experiments/<megapose-coarse-run>/  — MegaPose weights
#   $MEGAPOSE_MESHES/<class>.(obj|ply)                     — one per YOLO class
export HAPPYPOSE_WEIGHTS=/path/to/models/happypose
export MEGAPOSE_MESHES=/path/to/models/megapose/meshes

# Single-cam:
docker compose -f docker/docker-compose.yml up megapose

# Multi-cam fan-out (/cam0 + /cam1):
MEGAPOSE_CAMERA_CONFIG=/ws/config/camera_config_2cam.yaml \
  docker compose -f docker/docker-compose.yml up megapose
```

### Multi-camera fan-out

Pass `camera_config:=<yaml>` to the launch file to spawn one LifecycleNode per camera declared in `perception_system.cameras`. Each instance gets its own namespace with all its topics prefixed. See [CLAUDE.md](../../../CLAUDE.md).

## Dependencies

- **ROS**: `rclpy`, `sensor_msgs`, `geometry_msgs`, `perception_msgs`, `cv_bridge`
- **Python (host)**: numpy, PyYAML
- **Python (megapose backend, Docker only)**: torch 2.6, torchvision 0.21, PyTorch3D v0.7.9, happypose (clone at `/opt/happypose`), panda3d, pin (pinocchio). Provisioned by `docker/Dockerfile` stage `cosypose-runtime` (shared with CosyPose). See [`memory/happypose_install_trap.md`](../../../.claude/projects/.../memory/happypose_install_trap.md) for the install recipe.
