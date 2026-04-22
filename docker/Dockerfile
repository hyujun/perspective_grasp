# syntax=docker/dockerfile:1.7
# =============================================================================
# perspective_grasp - ML Node Base Image
#
# Shared base for GPU-heavy Python ML nodes that need dependency isolation.
# Host runs: C++ nodes, camera drivers, ur5e controller, RViz2, YOLO
# Docker runs: FoundationPose, CosyPose + MegaPose (shared stage), BundleSDF, SAM2
#
# Usage:
#   COMPOSE_BAKE=true docker compose build       # Parallel build (recommended)
#   docker compose up foundationpose              # Start one service
#
# Build performance:
# * Requires BuildKit (default in Docker 23+). The `# syntax=` line above
#   activates the v1.7 frontend so `RUN --mount=type=cache,...` is honored.
# * apt and pip layers use cache mounts — the first build downloads
#   torch/kaolin/nvdiffrast wheels and apt packages once; subsequent
#   builds reuse them even when intermediate layers are invalidated.
# * `COMPOSE_BAKE=true` makes `docker compose build` use buildx bake under
#   the hood, so the four runtime stages build in parallel against the
#   shared `ml-base` parent (3-4x wall-clock speedup on first build).
# =============================================================================

# ---------------------------------------------------------------------------
# Stage 1: ros-ml-base — CUDA + ROS 2 Jazzy (minimal, no desktop)
# ---------------------------------------------------------------------------
FROM nvidia/cuda:12.6.3-cudnn-devel-ubuntu24.04 AS ros-ml-base

ENV DEBIAN_FRONTEND=noninteractive
ENV LANG=C.UTF-8
ENV LC_ALL=C.UTF-8
ENV ROS_DISTRO=jazzy

SHELL ["/bin/bash", "-c"]

# Disable apt's default post-install cache wipe so BuildKit cache mounts
# below actually retain downloaded .deb files between builds.
RUN rm -f /etc/apt/apt.conf.d/docker-clean \
    && echo 'Binary::apt::APT::Keep-Downloaded-Packages "true";' \
        > /etc/apt/apt.conf.d/keep-cache

# ---- ROS 2 Jazzy repository ----
RUN --mount=type=cache,target=/var/cache/apt,sharing=locked \
    --mount=type=cache,target=/var/lib/apt,sharing=locked \
    apt-get update && apt-get install -y --no-install-recommends \
        curl gnupg2 lsb-release software-properties-common ca-certificates \
    && curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
        -o /usr/share/keyrings/ros-archive-keyring.gpg \
    && echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
        http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" \
        > /etc/apt/sources.list.d/ros2.list

# ---- ROS 2 base + packages needed by ML nodes ----
# ros-base includes: rclpy, sensor_msgs, std_msgs, geometry_msgs, launch_ros
# Additional packages for ML node functionality:
#   cv-bridge:          Image format conversion (ROS <-> OpenCV/numpy)
#   tf2-ros:            Transform lookups (camera frame → base frame)
#   tf2-geometry-msgs:  Transform pose messages
#   image-transport:    Efficient image transport (compressed topics)
RUN --mount=type=cache,target=/var/cache/apt,sharing=locked \
    --mount=type=cache,target=/var/lib/apt,sharing=locked \
    apt-get update && apt-get install -y --no-install-recommends \
        ros-jazzy-ros-base \
        python3-colcon-common-extensions \
        python3-pip \
        build-essential \
        cmake \
        git \
        ros-jazzy-cv-bridge \
        ros-jazzy-tf2-ros \
        ros-jazzy-tf2-geometry-msgs \
        ros-jazzy-image-transport

# numpy is provided by python3-numpy (debian); OpenCV Python bindings come
# via ros-jazzy-cv-bridge. Skip pip installs here to avoid clashing with
# externally-managed system packages.

RUN echo "source /opt/ros/jazzy/setup.bash" >> /etc/bash.bashrc

# ---------------------------------------------------------------------------
# Stage 2: msgs-builder — build perception_msgs so ML nodes can use them
# ---------------------------------------------------------------------------
FROM ros-ml-base AS msgs-builder

RUN mkdir -p /ws/src
COPY packages/interfaces/perception_msgs /ws/src/perception_msgs

RUN source /opt/ros/jazzy/setup.bash \
    && cd /ws && colcon build --packages-select perception_msgs

# ---------------------------------------------------------------------------
# Stage 3: ml-base — final image with compiled msgs
# ---------------------------------------------------------------------------
FROM ros-ml-base AS ml-base

COPY --from=msgs-builder /ws/install /ws/install

RUN echo "source /ws/install/setup.bash" >> /etc/bash.bashrc

WORKDIR /ws
CMD ["bash"]

# ---------------------------------------------------------------------------
# Stage 4: foundationpose-runtime — ml-base + PyTorch + kaolin + nvdiffrast
#
# NVIDIA FoundationPose (https://github.com/NVlabs/FoundationPose) needs:
#   * torch / torchvision (CUDA 12.6 whls)
#   * kaolin (SDF + rendering helpers; headers baked against torch ABI)
#   * nvdiffrast (differentiable rasteriser; builds CUDA ops at install)
#   * trimesh + pyrender + open3d for mesh I/O, shape ops
#
# The FoundationPose python sources are cloned into /opt/FoundationPose
# and added to PYTHONPATH so `from estimater import FoundationPose` works
# from the ROS node's runtime PYTHONPATH.
#
# Separate stage so kaolin/nvdiffrast CUDA-op versions do not clash with
# SAM2 / CosyPose / BundleSDF. Build is heavy (~15 min on 3070 Ti) —
# layers are ordered to maximise cache reuse when iterating on the ROS
# node sources only.
# ---------------------------------------------------------------------------
FROM ml-base AS foundationpose-runtime

# Shadow two debian-owned Python packages with pip-managed copies in
# /usr/local so the main `pip install -r requirements.txt` below never
# has to uninstall them (debian installs lack pip RECORD files, so pip
# errors with `Cannot uninstall X, RECORD file not found`).
#   * numpy<2: cv_bridge in ros-jazzy is compiled against the NumPy 1.x
#     C ABI. Letting transitive deps pull numpy 2.x breaks cv2 at import.
#   * blinker>=1.9: open3d 0.19.0 → flask 3.x → blinker>=1.9, but
#     python3-blinker 1.7 is pre-installed by ros-jazzy-ros-base.
RUN --mount=type=cache,target=/root/.cache/pip,sharing=locked \
    pip3 install --break-system-packages --ignore-installed \
        "numpy<2" "blinker>=1.9"

# Native deps for kaolin / nvdiffrast / open3d / pyrender.
RUN --mount=type=cache,target=/var/cache/apt,sharing=locked \
    --mount=type=cache,target=/var/lib/apt,sharing=locked \
    apt-get update && apt-get install -y --no-install-recommends \
        libegl1 libgl1 libgles2 libglvnd0 libxrender1 libxi6 libxkbcommon0 \
        libsm6 libxext6 libglib2.0-0 libosmesa6-dev freeglut3-dev \
        ninja-build

# Pinned Python runtime — single resolution pass for determinism.
COPY docker/requirements-foundationpose.txt /tmp/requirements.txt
RUN --mount=type=cache,target=/root/.cache/pip,sharing=locked \
    pip3 install --break-system-packages -r /tmp/requirements.txt \
    && rm /tmp/requirements.txt

# FoundationPose sources — SHA-pinned clone (was @main). Weights are
# volume-mounted at runtime (see docker-compose FOUNDATIONPOSE_WEIGHTS).
RUN git clone https://github.com/NVlabs/FoundationPose.git /opt/FoundationPose \
    && git -C /opt/FoundationPose checkout e3d597b8c6b851d053094ebd6fa240191c5238f8
ENV PYTHONPATH=/opt/FoundationPose:${PYTHONPATH}

# Bake the ROS 2 node source + compile it into /ws/install so `ros2 launch`
# works immediately on container start. For dev iteration without rebuilding
# the image, use docker/docker-compose.dev.yml which re-mounts the source and
# re-runs colcon build on container start.
COPY packages/phase4_refinement/isaac_foundationpose_tracker /ws/src/isaac_foundationpose_tracker
RUN source /opt/ros/jazzy/setup.bash && source /ws/install/setup.bash \
    && cd /ws && colcon build --packages-select isaac_foundationpose_tracker

WORKDIR /ws
CMD ["bash"]

# ---------------------------------------------------------------------------
# Stage 5: cosypose-runtime — ml-base + PyTorch + PyTorch3D + happypose
#
# happypose (https://github.com/agimus-project/happypose) bundles both
# CosyPose (scene-level multi-object optimization) and MegaPose
# (zero-shot RGB-only refinement) under one install tree. We share a
# single stage across the `cosypose` and `megapose` docker-compose
# services because the heavy deps (torch / pytorch3d / panda3d) are
# identical — no reason to build twice.
#
# Dep risks (flag at first build):
#   * pytorch3d does not reliably publish cu126/torch 2.6 wheels, so
#     we source-build against the pinned torch above (slow: ~10 min).
#   * happypose is still pre-1.0 — upstream API drift may require
#     bumping the pin. Log `pip show happypose` in the node at load()
#     so divergences surface in container logs rather than at inference.
# ---------------------------------------------------------------------------
FROM ml-base AS cosypose-runtime

# Shadow apt numpy with a pip-managed 1.x install. cv_bridge in ros-jazzy is
# compiled against the NumPy 1.x C ABI; transitive deps (scikit-image /
# pandas / seaborn) otherwise pull numpy 2.x and break cv2 at import time.
RUN --mount=type=cache,target=/root/.cache/pip,sharing=locked \
    pip3 install --break-system-packages --ignore-installed \
        "numpy<2"

# Native deps for pytorch3d / panda3d / assimp-based mesh loaders.
RUN --mount=type=cache,target=/var/cache/apt,sharing=locked \
    --mount=type=cache,target=/var/lib/apt,sharing=locked \
    apt-get update && apt-get install -y --no-install-recommends \
        libegl1 libgl1 libgles2 libglvnd0 libxrender1 libxi6 libxkbcommon0 \
        libsm6 libxext6 libglib2.0-0 libosmesa6-dev freeglut3-dev \
        libassimp-dev ninja-build

# Pinned Python runtime — single resolution pass for determinism. Excludes
# happypose itself (installed --no-deps below, see next block) and
# pytorch3d (separate --no-build-isolation RUN below; its setup.py does
# `import torch` at metadata time, which breaks pip's isolated build env).
COPY docker/requirements-cosypose.txt /tmp/requirements.txt
RUN --mount=type=cache,target=/root/.cache/pip,sharing=locked \
    pip3 install --break-system-packages -r /tmp/requirements.txt \
    && rm /tmp/requirements.txt

# PyTorch3D — SHA-pinned source build (was @v0.7.9, first tag with torch
# 2.6 / cu126 support). --no-build-isolation so setup.py sees the torch
# pinned in the previous RUN; default build isolation spawns a fresh
# venv without torch and the install fails.
RUN --mount=type=cache,target=/root/.cache/pip,sharing=locked \
    pip3 install --break-system-packages --no-build-isolation \
        "pytorch3d @ git+https://github.com/facebookresearch/pytorch3d.git@33824be3cbc87a7dd1db0f6a9a9de9ac81b2d0ba"

# happypose: clone the full repo (with submodules disabled) instead of
# relying on `pip install git+URL` alone. The upstream pyproject uses uv's
# workspace model, so pip-via-git pulls only the outer shell — the real
# CosyPose + MegaPose code under happypose/pose_estimators/** gets skipped.
# We do BOTH:
#   1. `pip install --no-deps git+URL@<SHA>` so `importlib.metadata.metadata
#      ("happypose")` resolves (happypose/__init__.py reads version + author
#      from dist-info at import time; without the dist-info every
#      `import happypose.*` raises PackageNotFoundError).
#   2. clone the tree into /opt/happypose so the real modules are on disk.
#   3. install the cosypose subproject (pybind11 C++ ext, sets up the
#      top-level `cosypose` namespace the backend uses).
#   4. add /opt/happypose to PYTHONPATH so happypose.* resolves to the
#      full source tree instead of the pip-installed shell.
# See memory/happypose_install_trap.md for the debugging trail.
# SHA-pinned (was @main, upstream last touched 2023-09).
# torchtnt / xarray / scikit-learn are pyproject-declared but unused by the
# CosyPose + MegaPose modules; --no-deps keeps the resolver in a pinned corner.
RUN --mount=type=cache,target=/root/.cache/pip,sharing=locked \
    pip3 install --break-system-packages --no-deps \
        "git+https://github.com/agimus-project/happypose.git@2f3d2ebbf17d20157a5a3f7821f62876bf594b03"

# No `--recurse-submodules`: happypose's `.gitmodules` references paths
# (`cosypose/deps/*`) that no longer exist in the repo, so the flag just
# wastes time probing 404s. The bits happypose actually uses at runtime
# live in `happypose/pose_estimators/**`, which is regular source.
RUN git clone https://github.com/agimus-project/happypose.git /opt/happypose \
    && git -C /opt/happypose checkout 2f3d2ebbf17d20157a5a3f7821f62876bf594b03

# cosypose subproject: has a C++ ext (cosypose_cext) and sets up the
# top-level `cosypose` namespace. The real project sits at the repo root
# (`/opt/happypose/cosypose/setup.py`); the `happypose/pose_estimators/
# cosypose/` path under the inner package is only a docstring stub
# (`__init__.py` + `placeholder.py`) at this pinned SHA.
#
# Two env-var gotchas the cosypose setup.py inherits from its conda-centric
# upstream:
#   * Line 10 does `os.environ['CXX'] = os.environ.get('GXX', '')`, which
#     clobbers CXX to empty when GXX is unset (the conda `compilers` pkg
#     sets GXX; plain Docker does not). An empty CXX makes distutils build
#     a compiler with empty `compiler_cxx`, and torch cpp_extension's
#     `_check_abi` then fails with `IndexError: list index out of range`.
#     Export GXX=g++ (and CXX=g++ defensively) so both paths resolve.
#   * --no-build-isolation because setup.py does `from torch.utils.
#     cpp_extension import CppExtension` at module top — pip's default
#     isolated build env has no torch (same trap as pytorch3d).
RUN --mount=type=cache,target=/root/.cache/pip,sharing=locked \
    cd /opt/happypose/cosypose \
    && CXX=g++ GXX=g++ pip3 install --break-system-packages --no-deps --no-build-isolation -e .

ENV PYTHONPATH=/opt/happypose:${PYTHONPATH}

# happypose drives its data-tree discovery off this env var; node params
# (`dataset_dir`) override it per-invocation when set.
ENV HAPPYPOSE_DATA_DIR=/ws/models/happypose

# Bake BOTH ROS 2 node sources (cosypose_scene_optimizer + megapose_ros2_wrapper)
# into /ws/install — this single stage serves the `cosypose` and `megapose`
# compose services, and either launch target must work without a runtime build.
COPY packages/phase4_refinement/cosypose_scene_optimizer /ws/src/cosypose_scene_optimizer
COPY packages/phase4_refinement/megapose_ros2_wrapper /ws/src/megapose_ros2_wrapper
RUN source /opt/ros/jazzy/setup.bash && source /ws/install/setup.bash \
    && cd /ws && colcon build --packages-select cosypose_scene_optimizer megapose_ros2_wrapper

WORKDIR /ws
CMD ["bash"]

# ---------------------------------------------------------------------------
# Stage 6: bundlesdf-runtime — ml-base + PyTorch + kaolin + PyTorch3D
#
# NVlabs BundleSDF (https://github.com/NVlabs/BundleSDF) builds a
# neural SDF + pose graph for unknown objects from sequential RGB-D
# + mask inputs. Needs:
#   * torch / torchvision (CUDA 12.6 wheels) — same pin as other stages
#   * kaolin 0.17 (mesh + SDF helpers; headers bound to torch ABI)
#   * nvdiffrast 0.3.3 (differentiable rasteriser; used by NeRF renderer)
#   * PyTorch3D v0.7.9 (source-build against torch 2.6 / cu126)
#   * Open3D (mesh + point-cloud ops) — pip wheel
#   * trimesh / pyrender / imageio / ruamel.yaml / kornia / pyrender /
#     omegaconf — runtime utilities BundleSDF imports directly
#
# The BundleSDF python sources are cloned into /opt/BundleSDF and
# that path is added to PYTHONPATH so `from bundlesdf import BundleSdf`
# works from the ROS node's runtime PYTHONPATH. The BundleTrack
# pybind11 / CUDA extension is NOT pre-built here — it compiles on
# first import inside the container so the image build stays bounded.
# Weights + NeRF configs are volume-mounted at runtime (see
# docker-compose BUNDLESDF_WEIGHTS).
#
# Dep risk notes (see memory/bundlesdf_next_steps.md + happypose_install_trap.md):
#   * pin numpy<2 in every layer — sklearn / imageio etc. drag numpy 2
#     through transitive resolution, breaking cv_bridge's NumPy 1.x ABI.
#   * BundleSDF's .gitmodules points at external repos that may 404;
#     clone without --recurse-submodules, same workaround CosyPose used.
#   * skip torchtnt / scikit-learn pyproject extras unless a concrete
#     import demands them (they force torch upgrades / numpy 2.x).
# ---------------------------------------------------------------------------
FROM ml-base AS bundlesdf-runtime

# Shadow two debian-owned Python packages with pip-managed copies in
# /usr/local (debian installs lack pip RECORD files — pip errors with
# `Cannot uninstall X, RECORD file not found` if it tries to upgrade).
#   * numpy<2: cv_bridge NumPy 1.x ABI.
#   * blinker>=1.9: open3d 0.19.0 → flask 3.x → blinker>=1.9, vs the
#     python3-blinker 1.7 that ros-jazzy-ros-base drops in.
RUN --mount=type=cache,target=/root/.cache/pip,sharing=locked \
    pip3 install --break-system-packages --ignore-installed \
        "numpy<2" "blinker>=1.9"

# Native deps: kaolin / nvdiffrast / open3d / pyrender / BundleSDF C++ ext.
RUN --mount=type=cache,target=/var/cache/apt,sharing=locked \
    --mount=type=cache,target=/var/lib/apt,sharing=locked \
    apt-get update && apt-get install -y --no-install-recommends \
        libegl1 libgl1 libgles2 libglvnd0 libxrender1 libxi6 libxkbcommon0 \
        libsm6 libxext6 libglib2.0-0 libosmesa6-dev freeglut3-dev \
        libeigen3-dev libboost-all-dev libyaml-cpp-dev libpcl-dev \
        ninja-build

# Pinned Python runtime — single resolution pass for determinism. Excludes
# pytorch3d (separate --no-build-isolation RUN below; its setup.py does
# `import torch` at metadata time, which breaks pip's isolated build env).
COPY docker/requirements-bundlesdf.txt /tmp/requirements.txt
RUN --mount=type=cache,target=/root/.cache/pip,sharing=locked \
    pip3 install --break-system-packages -r /tmp/requirements.txt \
    && rm /tmp/requirements.txt

# PyTorch3D — SHA-pinned source build (was @v0.7.9, first tag with torch
# 2.6 / cu126 support). --no-build-isolation so setup.py sees the torch
# pinned in the previous RUN; default build isolation spawns a fresh
# venv without torch and the install fails.
RUN --mount=type=cache,target=/root/.cache/pip,sharing=locked \
    pip3 install --break-system-packages --no-build-isolation \
        "pytorch3d @ git+https://github.com/facebookresearch/pytorch3d.git@33824be3cbc87a7dd1db0f6a9a9de9ac81b2d0ba"

# BundleSDF sources — SHA-pinned clone. Weights are volume-mounted at
# runtime (see docker-compose BUNDLESDF_WEIGHTS). No --recurse-submodules:
# BundleSDF's .gitmodules points at external repos that may 404 (same trap
# as happypose).
RUN git clone https://github.com/NVlabs/BundleSDF.git /opt/BundleSDF \
    && git -C /opt/BundleSDF checkout ffa67d425240b5b76d2e387a7dd3d3735a7cf1a1
ENV PYTHONPATH=/opt/BundleSDF:${PYTHONPATH}

# Bake the ROS 2 node source + compile it into /ws/install. See
# docker/docker-compose.dev.yml for the dev bind-mount workflow.
COPY packages/phase4_refinement/bundlesdf_unknown_tracker /ws/src/bundlesdf_unknown_tracker
RUN source /opt/ros/jazzy/setup.bash && source /ws/install/setup.bash \
    && cd /ws && colcon build --packages-select bundlesdf_unknown_tracker

WORKDIR /ws
CMD ["bash"]

# ---------------------------------------------------------------------------
# Stage 7: sam2-runtime — ml-base + PyTorch + Meta SAM2
#
# Separate stage so SAM2's PyTorch version does not clash with other
# Phase 4 services (kaolin/nvdiffrast/PyTorch3D have their own pins).
# ---------------------------------------------------------------------------
FROM ml-base AS sam2-runtime

# Pin numpy<2 — cv_bridge/cv2 shipped by ROS Jazzy is built against the
# NumPy 1.x C ABI; letting pip pull NumPy 2.x breaks cv2 at import time.
# Install this FIRST (with --ignore-installed) so the debian-managed
# numpy 1.26 gets shadowed by a pip-managed 1.x in /usr/local, and later
# torch / sam2 installs pick it up without attempting their own upgrade.
RUN --mount=type=cache,target=/root/.cache/pip,sharing=locked \
    pip3 install --break-system-packages --ignore-installed \
        "numpy<2"

# Pinned Python runtime — single resolution pass for determinism. Pulls
# torch 2.6 + cu126 and SAM2 @ SHA (was @main, upstream last touched 2024-12).
COPY docker/requirements-sam2.txt /tmp/requirements.txt
RUN --mount=type=cache,target=/root/.cache/pip,sharing=locked \
    pip3 install --break-system-packages -r /tmp/requirements.txt \
    && rm /tmp/requirements.txt

# Bake the ROS 2 node source + compile it into /ws/install. See
# docker/docker-compose.dev.yml for the dev bind-mount workflow.
COPY packages/phase4_refinement/sam2_instance_segmentor /ws/src/sam2_instance_segmentor
RUN source /opt/ros/jazzy/setup.bash && source /ws/install/setup.bash \
    && cd /ws && colcon build --packages-select sam2_instance_segmentor

WORKDIR /ws
CMD ["bash"]
