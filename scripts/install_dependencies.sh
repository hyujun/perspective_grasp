#!/bin/bash
# =============================================================================
# install_dependencies.sh - Install all external dependencies for perspective_grasp
#
# Use this when ROS 2 Jazzy is already installed and you only need the
# additional C++/Python libraries. For a full Ubuntu 24.04 setup from scratch,
# use install_host.sh instead.
#
# Required dependencies by package:
#   teaser_icp_hybrid_registrator: Eigen3, PCL, TEASER++ (optional)
#   cross_camera_associator:       Eigen3
#   pcl_merge_node:                PCL, MPI
#   perception_debug_visualizer:   OpenCV
#   multi_camera_calibration:      OpenCV, Eigen3, Ceres (optional), std_srvs
#   pose_filter_cpp:               Eigen3, manif
#   pose_graph_smoother:           Eigen3, GTSAM (optional)
#   yolo_pcl_cpp_tracker:          PCL, Eigen3, ultralytics (YOLO)
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
    libfmt-dev \
    libpcl-dev \
    libeigen3-dev \
    libopencv-dev \
    libopenmpi-dev \
    libceres-dev

# ---- 2. C++ libraries from source ----
echo ""
echo "=== [2/4] TEASER++ + manif (from source) ==="

# TEASER++
if ! ldconfig -p | grep -q teaserpp; then
    echo "Building TEASER++..."
    TEASER_DIR=/tmp/TEASER-plusplus
    [ ! -d "$TEASER_DIR" ] && git clone --depth 1 https://github.com/MIT-SPARK/TEASER-plusplus.git "$TEASER_DIR"
    cd "$TEASER_DIR" && mkdir -p build && cd build
    cmake .. -DCMAKE_BUILD_TYPE=Release \
             -DBUILD_TESTS=OFF \
             -DBUILD_PYTHON_BINDINGS=OFF \
             -DBUILD_DOC=OFF
    make -j"$(nproc)"
    sudo make install
    sudo ldconfig
else
    echo "TEASER++ already installed."
fi

# manif
if [ ! -f /usr/local/include/manif/manif.h ]; then
    echo "Installing manif..."
    MANIF_DIR=/tmp/manif
    [ ! -d "$MANIF_DIR" ] && git clone --depth 1 https://github.com/artivis/manif.git "$MANIF_DIR"
    cd "$MANIF_DIR" && mkdir -p build && cd build
    cmake .. -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF -DBUILD_EXAMPLES=OFF
    sudo make install
else
    echo "manif already installed."
fi

# ---- 3. GTSAM ----
echo ""
echo "=== [3/4] GTSAM ==="
if ! dpkg -l | grep -q libgtsam-dev; then
    sudo add-apt-repository -y ppa:borglab/gtsam-release-4.2
    sudo apt update
    sudo apt install -y libgtsam-dev libgtsam-unstable-dev
else
    echo "GTSAM already installed."
fi

# ---- 4. Python ML packages ----
echo ""
echo "=== [4/4] Python: ultralytics (YOLO) ==="
pip3 install --user --break-system-packages ultralytics 2>/dev/null \
    || pip3 install --user ultralytics

echo ""
echo "============================================================"
echo " All dependencies installed successfully!"
echo ""
echo " Next: cd ~/ros2_ws/perspective_ws && ./src/perspective_grasp/build.sh"
echo "============================================================"
