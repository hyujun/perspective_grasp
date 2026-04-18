# Installation

Host setup for `perspective_grasp`. The workspace uses a **host + Docker hybrid**: C++ perception and YOLO run on the host, GPU-heavy ML nodes (FoundationPose, SAM2, CosyPose, BundleSDF) run in containers.

## Prerequisites

| Item | Requirement |
|------|-------------|
| OS | Ubuntu 24.04 |
| GPU | NVIDIA GPU, driver 560+, CUDA 12.4+ |
| ROS 2 | Jazzy Jalisco |
| Docker | Required only for Phase 4 ML nodes |
| Workspace path | `~/ros2_ws/perspective_ws/` (assumed throughout docs) |

> All five Phase 4 ML nodes (SAM2, FoundationPose, CosyPose, MegaPose, BundleSDF) are implemented with pluggable real+mock backends. SAM2 is live-verified; the other four are mock-smoke-tested and will produce real output once their Docker images are built and weights are in place.

## Clone

```bash
mkdir -p ~/ros2_ws/perspective_ws/src
cd ~/ros2_ws/perspective_ws/src
git clone https://github.com/hyujun/perspective_grasp.git
```

## Option A — Fresh Ubuntu 24.04 (full setup)

Installs NVIDIA driver, CUDA, ROS 2 Jazzy, all C++/Python deps, and Docker in one pass.

```bash
cd ~/ros2_ws/perspective_ws/src/perspective_grasp
chmod +x scripts/install_host.sh
./scripts/install_host.sh
```

The script is idempotent and covers:

1. NVIDIA Driver 560 + CUDA Toolkit 12.4
2. ROS 2 Jazzy Desktop + rosdep
3. ROS 2 packages (tf2, cv_bridge, pcl_conversions, image_transport, …)
4. C++ system libs (Eigen3, PCL, OpenCV, Ceres)
5. C++ from source (TEASER++, manif)
6. GTSAM (via `ppa:borglab/gtsam-release-4.2`)
7. Python venv at `~/ros2_ws/perspective_ws/.venv` (sibling to `build/install/log`) with `ultralytics` installed
8. Docker + `nvidia-container-toolkit`

> If the NVIDIA driver was installed on this run, **reboot** and re-run the script to pick up from the next step.
>
> After Docker installs, **log out and back in** so the new `docker` group membership takes effect.

## Option B — ROS 2 Jazzy already present (deps only)

Use this if Jazzy is already installed and you only need the extra libraries.

```bash
cd ~/ros2_ws/perspective_ws/src/perspective_grasp
chmod +x scripts/install_dependencies.sh
./scripts/install_dependencies.sh
```

Installs ROS 2 packages, system libs, TEASER++, manif, GTSAM, and ultralytics (into the workspace venv). Does **not** touch the NVIDIA driver, CUDA, or Docker.

## Python venv

Both install scripts create a Python virtual environment at `~/ros2_ws/perspective_ws/.venv` (alongside the colcon `build/install/log` directories) using `--system-site-packages`, and install host-side pip packages (`ultralytics`) there. This keeps pip-installed Python deps isolated from `~/.local` and `/usr/lib/python3/dist-packages`, while leaving ROS 2's apt-installed bindings (`rclpy`, `cv_bridge`, numpy 1.26.4) visible.

> **You must activate the venv before building (optional) or running any Python-backed node.** `build.sh` auto-activates it if present; for manual `ros2 launch` / `ros2 run`, source it yourself:
>
> ```bash
> source /opt/ros/jazzy/setup.bash
> source ~/ros2_ws/perspective_ws/install/setup.bash
> source ~/ros2_ws/perspective_ws/.venv/bin/activate
> ```
>
> Without the venv active, nodes that `import ultralytics` (e.g. `yolo_pcl_cpp_tracker`) will fail with `ModuleNotFoundError`.

The venv pins `numpy<2` so it stays ABI-compatible with `cv_bridge` (which was compiled against apt's numpy 1.26.4). `opencv-python` from PyPI is explicitly **not** installed — the ROS 2 stack already provides `cv2` via `ros-jazzy-cv-bridge`, and installing a second copy causes load-order conflicts.

## Docker images (Phase 4 ML stack)

The Phase 4 services share a base image `perspective_grasp/ml-base` built from [docker/Dockerfile](../docker/Dockerfile). Each GPU-heavy service has its own runtime stage when its dependency stack (PyTorch + CUDA ops) would otherwise collide with peers:

| Stage | Pins | Used by |
|---|---|---|
| `foundationpose-runtime` | torch 2.6 + kaolin 0.17 + nvdiffrast 0.3.3 + NVlabs/FoundationPose at `/opt/FoundationPose` | `foundationpose` |
| `cosypose-runtime` | torch 2.6 + PyTorch3D v0.7.9 + happypose at `/opt/happypose` | `cosypose`, `megapose` (shared) |
| `bundlesdf-runtime` | torch 2.6 + kaolin 0.17 + nvdiffrast 0.3.3 + PyTorch3D v0.7.9 + Open3D 0.18 + NVlabs/BundleSDF at `/opt/BundleSDF` | `bundlesdf` |
| `sam2-runtime` | torch 2.6 + Meta SAM2 | `sam2` |

```bash
cd ~/ros2_ws/perspective_ws/src/perspective_grasp
docker compose -f docker/docker-compose.yml build
```

Compose context is the repo root, so volume paths inside [docker/docker-compose.yml](../docker/docker-compose.yml) reference `../packages/phase4_refinement/<pkg>`. `network_mode: host` + `ipc: host` lets DDS in the container talk to host nodes directly.

### Model weights

Each Phase 4 service expects weights under a mount point. Either set an env var or drop weights in the default `models/<service>/` directory at repo root:

```bash
# FoundationPose expects two sub-dirs under this mount:
#   meshes/<class_name>.(obj|ply|stl)   — one file per YOLO class
#   weights/                            — FoundationPose refiner / scorer checkpoints
export FOUNDATIONPOSE_WEIGHTS=/path/to/models/foundationpose

# SAM2 checkpoint (sam2_hiera_large.pt)
export SAM2_WEIGHTS=/path/to/models/sam2

# happypose weights shared by CosyPose + MegaPose; meshes directory shared too
# (flat: <class>.(obj|ply), one file per YOLO class — .stl not supported).
export HAPPYPOSE_WEIGHTS=/path/to/models/happypose
export MEGAPOSE_MESHES=/path/to/models/megapose/meshes

# BundleSDF writes per-track SDF/debug output here; defaults (track/nerf configs)
# come from the cloned /opt/BundleSDF repo.
export BUNDLESDF_WEIGHTS=/path/to/models/bundlesdf
```

## Verification

After installation:

```bash
# GPU + driver
nvidia-smi

# CUDA toolkit
nvcc --version

# ROS 2
source /opt/ros/jazzy/setup.bash
ros2 doctor

# Docker + NVIDIA runtime (should print nvidia-smi output from inside a container)
docker run --rm --gpus all nvidia/cuda:12.4.0-base-ubuntu22.04 nvidia-smi
```

## Next

- [build.md](build.md) — compile the workspace
- [running.md](running.md) — launch the pipeline
