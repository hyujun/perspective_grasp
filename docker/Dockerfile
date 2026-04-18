# =============================================================================
# perspective_grasp - ML Node Base Image
#
# Shared base for GPU-heavy Python ML nodes that need dependency isolation.
# Host runs: C++ nodes, camera drivers, ur5e controller, RViz2, YOLO
# Docker runs: FoundationPose, CosyPose + MegaPose (shared stage), SAM2, BundleSDF
#
# Usage:
#   docker compose build              # Build all images
#   docker compose up foundationpose   # Start one service
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

# ---- ROS 2 Jazzy repository ----
RUN apt-get update && apt-get install -y --no-install-recommends \
        curl gnupg2 lsb-release software-properties-common ca-certificates \
    && curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
        -o /usr/share/keyrings/ros-archive-keyring.gpg \
    && echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
        http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" \
        > /etc/apt/sources.list.d/ros2.list \
    && rm -rf /var/lib/apt/lists/*

# ---- ROS 2 base + packages needed by ML nodes ----
# ros-base includes: rclpy, sensor_msgs, std_msgs, geometry_msgs, launch_ros
# Additional packages for ML node functionality:
#   cv-bridge:          Image format conversion (ROS <-> OpenCV/numpy)
#   tf2-ros:            Transform lookups (camera frame → base frame)
#   tf2-geometry-msgs:  Transform pose messages
#   image-transport:    Efficient image transport (compressed topics)
RUN apt-get update && apt-get install -y --no-install-recommends \
        ros-jazzy-ros-base \
        python3-colcon-common-extensions \
        python3-pip \
        build-essential \
        cmake \
        git \
        ros-jazzy-cv-bridge \
        ros-jazzy-tf2-ros \
        ros-jazzy-tf2-geometry-msgs \
        ros-jazzy-image-transport \
    && rm -rf /var/lib/apt/lists/*

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

# numpy<2 first (see SAM2 stage — same cv_bridge ABI constraint).
RUN pip3 install --no-cache-dir --break-system-packages --ignore-installed \
        "numpy<2"

# Native deps for kaolin / nvdiffrast / open3d / pyrender.
RUN apt-get update && apt-get install -y --no-install-recommends \
        libegl1 libgl1 libgles2 libglvnd0 libxrender1 libxi6 libxkbcommon0 \
        libsm6 libxext6 libglib2.0-0 libosmesa6-dev freeglut3-dev \
        ninja-build \
    && rm -rf /var/lib/apt/lists/*

# PyTorch with CUDA 12.6. Pinned to versions known to work with
# kaolin 0.17 / nvdiffrast 0.3.x as of 2026-04.
RUN pip3 install --no-cache-dir --break-system-packages \
        --index-url https://download.pytorch.org/whl/cu126 \
        torch==2.6.0 torchvision==0.21.0

# Mesh / rendering utilities FoundationPose pulls in at runtime.
RUN pip3 install --no-cache-dir --break-system-packages \
        trimesh==4.5.3 pyrender==0.1.45 open3d==0.18.0 \
        opencv-python-headless==4.10.0.84 \
        scipy==1.14.1 scikit-image==0.24.0 \
        imageio==2.36.0 omegaconf==2.3.0 \
        ruamel.yaml==0.18.6 joblib==1.4.2

# nvdiffrast + kaolin — compile against the torch we just installed.
RUN pip3 install --no-cache-dir --break-system-packages \
        git+https://github.com/NVlabs/nvdiffrast.git@v0.3.3
RUN pip3 install --no-cache-dir --break-system-packages \
        kaolin==0.17.0 \
        -f https://nvidia-kaolin.s3.us-east-2.amazonaws.com/torch-2.6.0_cu126.html

# FoundationPose sources — cloned once, put on PYTHONPATH. Weights are
# volume-mounted at runtime (see docker-compose FOUNDATIONPOSE_WEIGHTS).
RUN git clone --depth 1 https://github.com/NVlabs/FoundationPose.git \
        /opt/FoundationPose
ENV PYTHONPATH=/opt/FoundationPose:${PYTHONPATH}

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

# numpy<2 first (cv_bridge ABI — same constraint as foundationpose/sam2).
RUN pip3 install --no-cache-dir --break-system-packages --ignore-installed \
        "numpy<2"

# Native deps for pytorch3d / panda3d / assimp-based mesh loaders.
RUN apt-get update && apt-get install -y --no-install-recommends \
        libegl1 libgl1 libgles2 libglvnd0 libxrender1 libxi6 libxkbcommon0 \
        libsm6 libxext6 libglib2.0-0 libosmesa6-dev freeglut3-dev \
        libassimp-dev ninja-build \
    && rm -rf /var/lib/apt/lists/*

# PyTorch 2.6 + CUDA 12.6 — same pin the other Phase 4 stages use.
RUN pip3 install --no-cache-dir --break-system-packages \
        --index-url https://download.pytorch.org/whl/cu126 \
        torch==2.6.0 torchvision==0.21.0

# happypose runtime deps pulled ahead of happypose itself so source
# builds (pytorch3d) see the right numeric stack. pybind11 is a build-
# time dep for cosypose's C++ extension (cosypose_cext). `"numpy<2"`
# is re-asserted here because scikit-image / pandas / seaborn pull
# numpy 2.x through transitive resolution otherwise, which breaks
# cv_bridge at runtime (NumPy 1.x C ABI).
RUN pip3 install --no-cache-dir --break-system-packages \
        "numpy<2" \
        trimesh==4.5.3 \
        opencv-python-headless==4.10.0.84 \
        scipy==1.14.1 scikit-image==0.24.0 \
        imageio==2.36.0 omegaconf==2.3.0 \
        pandas==2.2.3 transforms3d==0.4.2 \
        panda3d==1.10.14 joblib==1.4.2 \
        pybullet==3.2.6 bokeh==3.6.0 plyfile==1.1 \
        pybind11==2.13.6 webdataset==0.2.100 \
        pypng==0.20220715.0 addict==2.4.0 meshcat==0.3.2 \
        roma==1.5.0 torchnet==0.0.4 seaborn==0.13.2

# PyTorch3D — source install against the torch above. Upstream skipped
# v0.7.8; v0.7.9 is the first tag shipping torch 2.6 / cu126 support.
RUN pip3 install --no-cache-dir --break-system-packages \
        "git+https://github.com/facebookresearch/pytorch3d.git@v0.7.9"

# happypose: clone the full repo (with submodules) instead of relying
# on `pip install git+URL` alone. The upstream pyproject uses uv's
# workspace model, so pip-via-git pulls only the outer shell — the
# real CosyPose + MegaPose code under happypose/pose_estimators/**
# gets skipped. We do BOTH:
#   1. `pip install --no-deps git+URL` so `importlib.metadata.metadata
#      ("happypose")` resolves (happypose/__init__.py reads version +
#      author from dist-info at import time; without the dist-info
#      every `import happypose.*` raises PackageNotFoundError).
#   2. clone the tree with submodules into /opt/happypose so the real
#      modules are available on disk.
#   3. install the cosypose subproject (has pybind11 C++ ext and sets
#      up the top-level `cosypose` namespace the backend uses).
#   4. add /opt/happypose to PYTHONPATH so happypose.* resolves to
#      the full source tree instead of the pip-installed shell.
# See memory/happypose_install_trap.md for the debugging trail.
RUN pip3 install --no-cache-dir --break-system-packages --no-deps \
        "git+https://github.com/agimus-project/happypose.git@main"

# Runtime deps happypose declares but `--no-deps` skipped above. pin
# is pinocchio's python binding (SE(3) maths); the cmeel-* helpers
# get pulled in transitively. Only deps happypose actually imports
# on the pose-estimation path go here — `torchtnt`, `xarray`,
# `scikit-learn` are pyproject-declared but unused by the CosyPose +
# MegaPose modules, and adding them destabilises the resolver (e.g.
# old-xarray → pandas<2 source build with missing pkg_resources).
# Add entries below only when a concrete import-time failure demands
# them, so the resolver stays in a pinned corner.
RUN pip3 install --no-cache-dir --break-system-packages \
        "numpy<2" \
        "pin>=2.7" simplejson pyyaml tqdm beautifulsoup4

# No `--recurse-submodules`: happypose's `.gitmodules` references
# paths (`cosypose/deps/*`) that no longer exist in the repo, so the
# flag just wastes time probing 404s. The bits happypose actually
# uses at runtime live in `happypose/pose_estimators/**`, which is
# regular source.
RUN git clone --depth 1 \
        https://github.com/agimus-project/happypose.git /opt/happypose

# cosypose subproject: has pybind11 C++ ext (cosypose_cext) and sets
# up the top-level `cosypose` namespace.
RUN cd /opt/happypose/happypose/pose_estimators/cosypose \
    && pip3 install --no-cache-dir --break-system-packages --no-deps -e .

ENV PYTHONPATH=/opt/happypose:${PYTHONPATH}

# happypose drives its data-tree discovery off this env var; node
# params (`dataset_dir`) override it per-invocation when set.
ENV HAPPYPOSE_DATA_DIR=/ws/models/happypose

WORKDIR /ws
CMD ["bash"]

# ---------------------------------------------------------------------------
# Stage 6: sam2-runtime — ml-base + PyTorch + Meta SAM2
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
RUN pip3 install --no-cache-dir --break-system-packages --ignore-installed \
        "numpy<2"

# PyTorch with CUDA 12.6 runtime. Versions pinned for reproducibility.
RUN pip3 install --no-cache-dir --break-system-packages \
        --index-url https://download.pytorch.org/whl/cu126 \
        torch==2.6.0 torchvision==0.21.0

# Meta's SAM2 (segment-anything-2). Installs hydra-core + iopath as deps.
RUN pip3 install --no-cache-dir --break-system-packages \
        "git+https://github.com/facebookresearch/sam2.git@main"

WORKDIR /ws
CMD ["bash"]
