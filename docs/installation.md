# Installation

Host setup for `perspective_grasp`. The workspace uses a **host + Docker hybrid**: C++ perception and YOLO run on the host, GPU-heavy ML nodes (FoundationPose, SAM2, CosyPose, BundleSDF) run in containers.

## Prerequisites

| Item | Requirement |
|------|-------------|
| OS | Ubuntu 24.04 |
| GPU | NVIDIA GPU + driver ≥560 (installed via `ubuntu-drivers autoinstall`; the host does **not** need a CUDA toolkit — each Phase 4 Docker stage bundles CUDA 12.6) |
| ROS 2 | Jazzy Jalisco |
| Docker | Required only for Phase 4 ML nodes |
| Workspace path | Any colcon workspace; this repo lives at `${ROS2_WS}/src/perspective_grasp/` |

> All five Phase 4 ML nodes (SAM2, FoundationPose, CosyPose, MegaPose, BundleSDF) are implemented with pluggable real+mock backends. SAM2 is live-verified; the other four are mock-smoke-tested and will produce real output once their Docker images are built and weights are in place.

## Workspace layout

Throughout these docs, `${ROS2_WS}` refers to **your** colcon workspace root — the directory that holds `build/`, `install/`, `log/`, and `.venv/`. Pick anything (e.g. `~/ros2_ws/perspective_ws`, `~/dev_ws`, `/opt/work`); the only fixed assumption is that this repo is cloned at `${ROS2_WS}/src/perspective_grasp/`. You can either export `ROS2_WS` in your shell and paste the commands verbatim, or substitute your path mentally.

```bash
export ROS2_WS=~/ros2_ws/perspective_ws   # example — use whatever path you prefer
```

The install/build/runtime shell scripts (`scripts/install_host.sh`, `scripts/install_dependencies.sh`, `build.sh`, `.env.live`) discover the workspace root from their own location, so they work with any layout that satisfies `${ROS2_WS}/src/perspective_grasp/`.

## Clone

```bash
mkdir -p ${ROS2_WS}/src
cd ${ROS2_WS}/src
git clone https://github.com/hyujun/perspective_grasp.git
```

## Option A — Fresh Ubuntu 24.04 (full setup)

Installs NVIDIA driver, ROS 2 Jazzy, all C++/Python deps, and Docker in one pass.

```bash
cd ${ROS2_WS}/src/perspective_grasp
chmod +x scripts/install_host.sh
./scripts/install_host.sh
```

The script is idempotent and refuses to run as root. It covers 8 steps:

0. Enable apt `universe` (nvidia-driver / libpcl-dev / libceres-dev live there) + base tools
1. NVIDIA driver ≥560 via `ubuntu-drivers autoinstall` (the script lets ubuntu-drivers pick the recommended variant for the GPU; reboot required after install — re-run the script once up)
2. ROS 2 Jazzy Desktop, registered via the canonical `ros2-apt-source` deb (handles upstream key rotations automatically) + rosdep
3. ROS 2 apt packages + system C++ libs (rclcpp/rclpy, **rmw-cyclonedds-cpp**, tf2, cv_bridge, image_transport, PCL, Eigen3, OpenCV, Ceres, openmpi)
4. C++ libs from source (TEASER++, manif, GTSAM 4.2.0)
5. Host Python venv at `${ROS2_WS}/.venv` via [`scripts/requirements-host.txt`](../scripts/requirements-host.txt) (pip runs with `--no-user` to keep transitive deps inside the venv — see memory note `pip_venv_shadow_trap`)
6. Docker + `nvidia-container-toolkit`
7. Model-weights directory scaffold under `models/<service>/`

> The Cyclone DDS RMW (`ros-jazzy-rmw-cyclonedds-cpp`) is **required** even though no package.xml lists it: the runtime forces `RMW_IMPLEMENTATION=rmw_cyclonedds_cpp` via `.env.live` to lockstep with the Phase 4 Docker containers. Without it, every host-side `ros2` command exits with "RMW implementation 'rmw_cyclonedds_cpp' is not installed" (CLAUDE.md anti-pattern (m)).

> The host CUDA toolkit (`nvcc`) is **not** installed. Each Phase 4 Docker stage bundles its own CUDA 12.6 — the host only needs the driver.
>
> GTSAM is always source-built: the borglab PPA does not publish for Ubuntu 24.04 (noble).

> If the NVIDIA driver was installed on this run, **reboot** and re-run the script to pick up from the next step.
>
> After Docker installs, **log out and back in** so the new `docker` group membership takes effect.

## Option B — ROS 2 Jazzy already present (deps only)

Use this if Jazzy is already installed and you only need the extra libraries.

```bash
cd ${ROS2_WS}/src/perspective_grasp
chmod +x scripts/install_dependencies.sh
./scripts/install_dependencies.sh
```

Installs ROS 2 packages, system libs, TEASER++, manif, GTSAM, and ultralytics (into the workspace venv). Does **not** touch the NVIDIA driver, CUDA, or Docker.

## RealSense cameras (optional)

If the host will drive Intel RealSense D-series cameras directly (Phase 1 input), run the dedicated installer **after** the host setup script has put `/opt/ros/jazzy` in place:

```bash
cd ${ROS2_WS}/src/perspective_grasp
./scripts/install_realsense.sh
```

It installs librealsense2 from Intel's apt repo (without the DKMS module — unsigned + Secure Boot incompatible), pulls `ros-jazzy-realsense2-camera{,-msgs,-description}`, reloads udev, and force-loads `uvcvideo` at boot. Verified end-state is `ros2 launch realsense2_camera rs_launch.py camera_namespace:=/ camera_name:=camera pointcloud.enable:=true align_depth.enable:=true`. Phase 1 expects the driver's native cloud topic `camera/depth/color/points` — no remapping required. See [running.md § Camera drivers](running.md#camera-drivers) for the multi-camera variants.

## Python venv — layered, not isolated

Both install scripts create a Python virtual environment at `${ROS2_WS}/.venv` (sibling to `build/install/log`) using `python3 -m venv --system-site-packages`. The pinned deps come from [`scripts/requirements-host.txt`](../scripts/requirements-host.txt) — currently `numpy<2` and `ultralytics`.

> **This is a layer, not an isolation boundary.** `--system-site-packages` means the venv *inherits* everything under `/usr/lib/python3/dist-packages/` (apt's rclpy / cv_bridge / numpy 1.26.4) and `~/.local/lib/python3.12/site-packages/`. The venv only adds or shadows packages on top.
>
> This is intentional: ROS 2 Python bindings (`rclpy`, `cv_bridge`) ship as apt debs and are not pip-installable, so a fully isolated venv would break ROS imports.

**You must activate the venv before building or running any Python-backed node.** `build.sh` auto-activates it if present; for manual `ros2 launch` / `ros2 run`, the recommended path is to source `.env.live` (handles ROS + workspace + Cyclone DDS — see next section) then layer the venv on top:

```bash
source ${ROS2_WS}/src/perspective_grasp/.env.live
source ${ROS2_WS}/.venv/bin/activate
```

Without the venv active, nodes that `import ultralytics` (e.g. `yolo_pcl_cpp_tracker`) will fail with `ModuleNotFoundError`.

## Host shell env — `.env.live`

Host terminals that talk to the Phase 4 Docker stack must match the container DDS settings or sensor-sized topics (Image, DetectionArray, SegmentationArray) silently drop on the wire. The repo ships a sourceable env file that pins all of it in one shot:

```bash
cd ${ROS2_WS}/src/perspective_grasp
source .env.live
# [pg-live] ws=${ROS2_WS}  rmw=rmw_cyclonedds_cpp
```

`.env.live` does four things:

1. `source /opt/ros/jazzy/setup.bash` and (if built) `<ws>/install/setup.bash`.
2. `export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp` — must match the container default.
3. `export CYCLONEDDS_URI=file://<repo>/packages/bringup/perception_bringup/config/cyclonedds_localhost.xml` — forces localhost-unicast peer discovery so SPDP doesn't wander off-box.
4. `cd` to the workspace root.

Verified live: SAM2 / RealSense host↔container topic flow needs all three of (host RMW=cyclone, container RMW=cyclone, both pointing at a localhost-peers XML). Mixing Fast DDS on the host with Cyclone in the container shows up as "topic listed by `ros2 topic list` but never echoes" — see [debugging.md § 4.9](debugging.md#49-phase-4-node-runs-but-host-doesnt-see-its-topics).

The venv must be sourced **on top** of `.env.live` for any node that imports ultralytics:

```bash
source .env.live
source ${ROS2_WS}/.venv/bin/activate
```

### Why numpy<2 and no opencv-python

- **`numpy<2`**: `cv_bridge` in ros-jazzy is compiled against the NumPy 1.x C ABI. Letting pip transitive resolution pull NumPy 2.x breaks `cv2` at import time. The pin applies to the venv; apt's numpy 1.26.4 remains visible via `--system-site-packages` and satisfies the constraint.
- **no `opencv-python`**: the ROS 2 stack already provides `cv2` via `ros-jazzy-cv-bridge`. A second pip-installed OpenCV causes load-order conflicts. The install scripts explicitly `pip uninstall opencv-python` after installing ultralytics, in case a transitive dep pulled it in.

## Docker images (Phase 4 ML stack)

The Phase 4 services share a base image `perspective_grasp/ml-base` built from [docker/Dockerfile](../docker/Dockerfile). Each GPU-heavy service has its own runtime stage when its dependency stack (PyTorch + CUDA ops) would otherwise collide with peers. Pins live in per-stage requirements files that Docker `COPY`s into each stage:

| Stage | Pinned by | Used by |
|---|---|---|
| `foundationpose-runtime` | [`docker/requirements-foundationpose.txt`](../docker/requirements-foundationpose.txt) + SHA-pinned `NVlabs/FoundationPose` at `/opt/FoundationPose` | `foundationpose` |
| `cosypose-runtime` | [`docker/requirements-cosypose.txt`](../docker/requirements-cosypose.txt) + SHA-pinned `agimus-project/happypose` at `/opt/happypose` | `cosypose`, `megapose` (shared) |
| `bundlesdf-runtime` | [`docker/requirements-bundlesdf.txt`](../docker/requirements-bundlesdf.txt) + SHA-pinned `NVlabs/BundleSDF` at `/opt/BundleSDF` | `bundlesdf` |
| `sam2-runtime` | [`docker/requirements-sam2.txt`](../docker/requirements-sam2.txt) (SHA-pinned Meta SAM2) | `sam2` |

All six git references (nvdiffrast, pytorch3d, happypose, sam2, FoundationPose, BundleSDF) are pinned to specific commit SHAs for reproducibility — search for `git checkout` / `@<sha>` in [docker/Dockerfile](../docker/Dockerfile) and the per-stage requirements files for the current values.

```bash
cd ${ROS2_WS}/src/perspective_grasp
docker compose -f docker/docker-compose.yml build
```

Compose context is the repo root, so volume paths inside [docker/docker-compose.yml](../docker/docker-compose.yml) reference `../packages/phase4_refinement/<pkg>`. `network_mode: host` + `ipc: host` lets DDS in the container talk to host nodes directly.

### Prod vs dev Docker workflow

Two compose files separate the two workflows:

| File | Use when | Source at `/ws/src/<pkg>` | On container start |
|---|---|---|---|
| `docker-compose.yml` (prod) | Deploying the workspace as-built | Baked into image at build time (COPY + colcon build) | Just `ros2 launch <pkg> <file>.launch.py` |
| `+ docker-compose.dev.yml` (dev) | Iterating on a Phase 4 node locally | Bind-mounted from `packages/phase4_refinement/<pkg>` | Re-runs `colcon build --packages-select <pkg>` then launch |

```bash
# Prod: no bind mount, no runtime colcon — fastest startup.
docker compose -f docker/docker-compose.yml up foundationpose

# Dev: bind-mount + on-start colcon build so local edits are picked up.
docker compose -f docker/docker-compose.yml \
               -f docker/docker-compose.dev.yml up foundationpose
```

Editing a Python file under `packages/phase4_refinement/<pkg>/` and restarting the dev container is enough — no image rebuild. Only `docker/requirements-<stage>.txt` changes (Python pins) require a rebuild.

### Image tag policy (REGISTRY + IMAGE_TAG)

`docker-compose.yml` templates both the registry namespace and the image tag, so the same compose file drives local builds, staging pulls, and production:

```bash
# Local build (default: REGISTRY=perspective_grasp, IMAGE_TAG=latest)
docker compose -f docker/docker-compose.yml build

# Apply a reproducible :YYYYMMDD-<git-sha> tag on top of :latest
./docker/tag.sh

# Push to a real registry (must override REGISTRY first)
export REGISTRY=ghcr.io/myorg/perspective_grasp
./docker/tag.sh --push
```

On a fresh production host:

```bash
export REGISTRY=ghcr.io/myorg/perspective_grasp
export IMAGE_TAG=2026-04-21-a8f21f2
docker compose -f docker/docker-compose.yml pull
docker compose -f docker/docker-compose.yml up -d
```

Record the `IMAGE_TAG` value that went to production — it's the rollback coordinate.

### Model weights

`install_host.sh` step 7 creates the `models/<service>/` scaffold. Per-service directory layout, file-naming contracts, and download instructions (SAM2 / FoundationPose / CosyPose+MegaPose / BundleSDF): see [model_assets.md](./model_assets.md).

Quick env-var reference for redirecting any service to an external disk:

```bash
export SAM2_WEIGHTS=/path/to/models/sam2
export FOUNDATIONPOSE_WEIGHTS=/path/to/models/foundationpose
export HAPPYPOSE_WEIGHTS=/path/to/models/happypose          # CosyPose + MegaPose share this
export MEGAPOSE_MESHES=/path/to/models/megapose/meshes
export BUNDLESDF_WEIGHTS=/path/to/models/bundlesdf
```

Unset → docker-compose falls back to `<repo>/models/<service>/`.

## Verification

After installation:

```bash
# GPU + driver (host CUDA toolkit is intentionally not installed — Docker bundles it)
nvidia-smi

# ROS 2
source /opt/ros/jazzy/setup.bash
ros2 doctor

# Docker + NVIDIA runtime (should print nvidia-smi output from inside a container)
docker run --rm --gpus all nvidia/cuda:12.6.3-base-ubuntu24.04 nvidia-smi

# Host venv + numpy 1.x ABI (should import without error)
source ${ROS2_WS}/.venv/bin/activate
python -c "import numpy, ultralytics, torch, cv_bridge; print('ok:', numpy.__version__, 'torch:', torch.__version__)"

# RealSense (only if install_realsense.sh was run — should list connected cams)
rs-enumerate-devices -s
ls /opt/ros/jazzy/share/realsense2_camera/launch/rs_launch.py
```

## Next

- [build.md](build.md) — compile the workspace
- [running.md](running.md) — launch the pipeline
