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

> Phase 4 nodes are currently **stubs** (launch files + skeletons, no inference). Installing the Docker stack is still useful for bringing up the runtime, but expect empty topic output until each node is implemented.

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
7. Python: `ultralytics` (YOLO)
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

Installs ROS 2 packages, system libs, TEASER++, manif, GTSAM, and ultralytics. Does **not** touch the NVIDIA driver, CUDA, or Docker.

## Docker images (Phase 4 ML stack)

The Phase 4 services share a base image `perspective_grasp/ml-base` built from [docker/Dockerfile](../docker/Dockerfile). SAM2 uses a dedicated `sam2-runtime` target to pin a compatible PyTorch version.

```bash
cd ~/ros2_ws/perspective_ws/src/perspective_grasp
docker compose -f docker/docker-compose.yml build
```

Compose context is the repo root, so volume paths inside [docker/docker-compose.yml](../docker/docker-compose.yml) reference `../packages/phase4_refinement/<pkg>`. `network_mode: host` + `ipc: host` lets DDS in the container talk to host nodes directly.

### Model weights

Each Phase 4 service expects weights under a mount point. Either set an env var or drop weights in the default `models/<service>/` directory at repo root:

```bash
export FOUNDATIONPOSE_WEIGHTS=/path/to/foundationpose/weights
export SAM2_WEIGHTS=/path/to/sam2/weights
export COSYPOSE_WEIGHTS=/path/to/cosypose/weights
export BUNDLESDF_WEIGHTS=/path/to/bundlesdf/weights
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
