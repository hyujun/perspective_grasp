# =============================================================================
# perspective_grasp - 6D Pose Estimation & Manipulation System
# Multi-stage Docker build: Ubuntu 24.04 + ROS 2 Jazzy + CUDA 12.4
# =============================================================================

# ---------------------------------------------------------------------------
# Stage 1: Base — OS + CUDA + ROS 2 Jazzy
# ---------------------------------------------------------------------------
FROM nvidia/cuda:12.4.1-cudnn-devel-ubuntu24.04 AS base

ENV DEBIAN_FRONTEND=noninteractive
ENV LANG=C.UTF-8
ENV LC_ALL=C.UTF-8
ENV ROS_DISTRO=jazzy

SHELL ["/bin/bash", "-c"]

# ---- ROS 2 Jazzy repository setup ----
RUN apt-get update && apt-get install -y --no-install-recommends \
        curl gnupg2 lsb-release software-properties-common ca-certificates \
    && curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
        -o /usr/share/keyrings/ros-archive-keyring.gpg \
    && echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
        http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" \
        > /etc/apt/sources.list.d/ros2.list \
    && rm -rf /var/lib/apt/lists/*

# ---- Core system + ROS 2 packages ----
RUN apt-get update && apt-get install -y --no-install-recommends \
        # ROS 2 Jazzy desktop (includes rclcpp, rclpy, rviz2, etc.)
        ros-jazzy-desktop \
        # Build tools
        python3-colcon-common-extensions \
        python3-rosdep \
        python3-pip \
        build-essential \
        cmake \
        git \
        # ROS 2 additional packages
        ros-jazzy-tf2-ros \
        ros-jazzy-tf2-eigen \
        ros-jazzy-tf2-sensor-msgs \
        ros-jazzy-tf2-geometry-msgs \
        ros-jazzy-rclcpp-action \
        ros-jazzy-rclcpp-lifecycle \
        ros-jazzy-cv-bridge \
        ros-jazzy-pcl-conversions \
        ros-jazzy-message-filters \
        ros-jazzy-image-transport \
        ros-jazzy-diagnostic-msgs \
        # System libraries
        libfmt-dev \
        libpcl-dev \
        libeigen3-dev \
        libopencv-dev \
        libopenmpi-dev \
    && rm -rf /var/lib/apt/lists/*

# ---- rosdep init ----
RUN rosdep init || true && rosdep update --rosdistro=$ROS_DISTRO

# ---------------------------------------------------------------------------
# Stage 2: Dependencies — TEASER++, manif, GTSAM (from source / PPA)
# ---------------------------------------------------------------------------
FROM base AS deps

# ---- TEASER++ ----
RUN git clone --depth 1 https://github.com/MIT-SPARK/TEASER-plusplus.git /tmp/teaser \
    && cd /tmp/teaser && mkdir build && cd build \
    && cmake .. -DCMAKE_BUILD_TYPE=Release \
                -DBUILD_TESTS=OFF \
                -DBUILD_PYTHON_BINDINGS=OFF \
                -DBUILD_DOC=OFF \
    && make -j"$(nproc)" && make install && ldconfig \
    && rm -rf /tmp/teaser

# ---- manif (header-only SE(3) library) ----
RUN git clone --depth 1 https://github.com/artivis/manif.git /tmp/manif \
    && cd /tmp/manif && mkdir build && cd build \
    && cmake .. -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF -DBUILD_EXAMPLES=OFF \
    && make install \
    && rm -rf /tmp/manif

# ---- GTSAM ----
RUN add-apt-repository -y ppa:borglab/gtsam-release-4.2 \
    && apt-get update \
    && apt-get install -y --no-install-recommends libgtsam-dev libgtsam-unstable-dev \
    && rm -rf /var/lib/apt/lists/*

# ---- Ceres Solver (optional, for multi_camera_calibration joint optimization) ----
RUN apt-get update && apt-get install -y --no-install-recommends \
        libceres-dev \
    && rm -rf /var/lib/apt/lists/*

# ---------------------------------------------------------------------------
# Stage 3: Python ML packages
# ---------------------------------------------------------------------------
FROM deps AS ml

RUN pip3 install --no-cache-dir --break-system-packages \
        ultralytics

# ---------------------------------------------------------------------------
# Stage 4: Workspace
# ---------------------------------------------------------------------------
FROM ml AS workspace

# Create workspace structure matching CLAUDE.md paths
RUN mkdir -p /home/junho/ros2_ws/perspective_ws/src

WORKDIR /home/junho/ros2_ws/perspective_ws/src

# Copy source (use .dockerignore to exclude build artifacts)
COPY . perspective_grasp/

# Build the workspace
WORKDIR /home/junho/ros2_ws/perspective_ws
RUN source /opt/ros/jazzy/setup.bash \
    && colcon build \
        --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo \
        --parallel-workers "$(( $(nproc) > 2 ? $(nproc) - 2 : 1 ))"

# ---------------------------------------------------------------------------
# Runtime setup
# ---------------------------------------------------------------------------

# Source ROS 2 + workspace on every shell
RUN echo "source /opt/ros/jazzy/setup.bash" >> /etc/bash.bashrc \
    && echo "source /home/junho/ros2_ws/perspective_ws/install/setup.bash" >> /etc/bash.bashrc

WORKDIR /home/junho/ros2_ws/perspective_ws
CMD ["bash"]
