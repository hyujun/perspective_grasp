#!/bin/bash
# =============================================================================
# install_dependencies.sh - Install external deps only (ROS 2 already present)
#
# Use this when ROS 2 Jazzy is already installed and you only need the
# additional C++/Python libraries. For a full Ubuntu 24.04 setup from
# scratch, use install_host.sh instead.
#
# External dependencies by package:
#   perception_msgs                : (pure msgs, no externals)
#   teaser_icp_hybrid_registrator  : Eigen3, PCL, TEASER++ (optional)
#   yolo_pcl_cpp_tracker           : PCL, Eigen3, ultralytics (YOLO)
#   cross_camera_associator        : Eigen3
#   pcl_merge_node                 : PCL, MPI
#   pose_filter_cpp                : Eigen3, manif
#   pose_graph_smoother            : Eigen3, GTSAM (optional)
#   multi_camera_calibration       : OpenCV, Eigen3, Ceres (optional), std_srvs
#   perception_debug_visualizer    : OpenCV, image_transport, cv_bridge
#   perception_meta_controller     : (ROS deps only)
#   perception_bringup             : (launch-only)
#   grasp_pose_planner             : sensor_msgs_py, rclpy
#   Phase 4 ML nodes               : packaged in Docker (docker/Dockerfile)
# =============================================================================
set -euo pipefail

echo "============================================================"
echo " perspective_grasp - Dependency Installation"
echo "============================================================"

# ---- 1. ROS 2 packages + system libraries ----
echo ""
echo "=== [1/4] ROS 2 packages + system libraries ==="
sudo apt update
sudo apt install -y \
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
    ros-jazzy-std-srvs \
    ros-jazzy-sensor-msgs-py \
    libfmt-dev \
    libpcl-dev \
    libeigen3-dev \
    libopencv-dev \
    libopenmpi-dev \
    libceres-dev

# ---- 2. C++ libraries from source (TEASER++, manif) ----
echo ""
echo "=== [2/4] TEASER++ + manif (from source) ==="

# TEASER++ (enables HAS_TEASERPP in teaser_icp_hybrid_registrator)
if ! ldconfig -p | grep -q teaserpp; then
    echo "Building TEASER++..."
    TEASER_DIR=/tmp/TEASER-plusplus
    [ ! -d "$TEASER_DIR" ] && git clone --depth 1 https://github.com/MIT-SPARK/TEASER-plusplus.git "$TEASER_DIR"
    (
        cd "$TEASER_DIR" && mkdir -p build && cd build
        cmake .. -DCMAKE_BUILD_TYPE=Release \
                 -DBUILD_TESTS=OFF \
                 -DBUILD_PYTHON_BINDINGS=OFF \
                 -DBUILD_DOC=OFF
        make -j"$(nproc)"
        sudo make install
        sudo ldconfig
    )
else
    echo "TEASER++ already installed."
fi

# manif (header-only; required by pose_filter_cpp)
if [ ! -f /usr/local/include/manif/manif.h ]; then
    echo "Installing manif..."
    MANIF_DIR=/tmp/manif
    [ ! -d "$MANIF_DIR" ] && git clone --depth 1 https://github.com/artivis/manif.git "$MANIF_DIR"
    (
        cd "$MANIF_DIR" && mkdir -p build && cd build
        cmake .. -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF -DBUILD_EXAMPLES=OFF
        sudo make install
    )
else
    echo "manif already installed."
fi

# ---- 3. GTSAM (enables HAS_GTSAM in pose_graph_smoother) ----
# Built from source: borglab PPA does not publish for Ubuntu 24.04 (noble).
echo ""
echo "=== [3/4] GTSAM (from source) ==="
if [ ! -f /usr/local/include/gtsam/base/Matrix.h ] && ! ldconfig -p | grep -q libgtsam.so; then
    echo "Building GTSAM 4.2.0..."
    GTSAM_DIR=/tmp/gtsam
    [ ! -d "$GTSAM_DIR" ] && git clone --depth 1 --branch 4.2.0 https://github.com/borglab/gtsam.git "$GTSAM_DIR"
    (
        cd "$GTSAM_DIR" && mkdir -p build && cd build
        cmake .. -DCMAKE_BUILD_TYPE=Release \
                 -DGTSAM_BUILD_TESTS=OFF \
                 -DGTSAM_BUILD_EXAMPLES_ALWAYS=OFF \
                 -DGTSAM_BUILD_UNSTABLE=ON \
                 -DGTSAM_USE_SYSTEM_EIGEN=ON \
                 -DGTSAM_BUILD_WITH_MARCH_NATIVE=OFF
        make -j"$(nproc)"
        sudo make install
        sudo ldconfig
    )
else
    echo "GTSAM already installed."
fi

# ---- 4. Python ML packages (host-side only) ----
echo ""
echo "=== [4/4] Python: ultralytics (YOLO) ==="
pip3 install --user --break-system-packages ultralytics 2>/dev/null \
    || pip3 install --user ultralytics

echo ""
echo "============================================================"
echo " All host-side dependencies installed successfully!"
echo ""
echo " Phase 4 ML nodes (FoundationPose, CosyPose, MegaPose, SAM2,"
echo " BundleSDF) run inside Docker — build those with:"
echo "   cd ~/ros2_ws/perspective_ws/src/perspective_grasp"
echo "   docker compose -f docker/docker-compose.yml build"
echo ""
echo " Next: cd ~/ros2_ws/perspective_ws && ./src/perspective_grasp/build.sh"
echo "============================================================"
