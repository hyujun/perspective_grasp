# =============================================================================
# perspective_grasp - ML Node Base Image
#
# Shared base for GPU-heavy Python ML nodes that need dependency isolation.
# Host runs: C++ nodes, camera drivers, ur5e controller, RViz2, YOLO
# Docker runs: FoundationPose, MegaPose/CosyPose, SAM2, BundleSDF
#
# Usage:
#   docker compose build              # Build all images
#   docker compose up foundationpose   # Start one service
# =============================================================================

# ---------------------------------------------------------------------------
# Stage 1: ros-ml-base — CUDA + ROS 2 Jazzy (minimal, no desktop)
# ---------------------------------------------------------------------------
FROM nvidia/cuda:12.4.1-cudnn-devel-ubuntu24.04 AS ros-ml-base

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

# ---- Python packages commonly needed by ML nodes ----
RUN pip3 install --no-cache-dir --break-system-packages \
        numpy \
        opencv-python-headless

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
