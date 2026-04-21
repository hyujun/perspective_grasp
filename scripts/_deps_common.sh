#!/bin/bash
# =============================================================================
# _deps_common.sh - Shared installation helpers.
#
# Sourced by install_host.sh (fresh Ubuntu path) and install_dependencies.sh
# (ROS 2 already present path). Each function is idempotent — safe to re-run.
#
# Functions defined here:
#   install_ros_apt_deps     ROS 2 Jazzy pkgs + system libs (apt only)
#   install_teaserpp         TEASER++ from source (HAS_TEASERPP gate)
#   install_manif            manif headers (pose_filter_cpp dep)
#   install_gtsam            GTSAM 4.2.0 from source (borglab PPA has no noble)
#   install_host_python      workspace venv + scripts/requirements-host.txt
#
# Callers are responsible for `set -euo pipefail` and any shell safety flags.
# This file intentionally does not set them so sourcing is side-effect-light.
# =============================================================================

_COMMON_SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck source=_venv.sh
source "$_COMMON_SCRIPT_DIR/_venv.sh"

install_ros_apt_deps() {
    echo ""
    echo "=== ROS 2 Jazzy packages + system libraries ==="
    sudo apt update
    sudo apt install -y \
        python3-colcon-common-extensions \
        python3-rosdep \
        python3-pip \
        build-essential \
        cmake \
        git \
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
}

install_teaserpp() {
    echo ""
    echo "=== TEASER++ (from source) ==="
    if ldconfig -p | grep -q teaserpp; then
        echo "TEASER++ already installed."
        return 0
    fi
    local src=/tmp/TEASER-plusplus
    [ ! -d "$src" ] && git clone --depth 1 https://github.com/MIT-SPARK/TEASER-plusplus.git "$src"
    (
        cd "$src" && mkdir -p build && cd build
        cmake .. -DCMAKE_BUILD_TYPE=Release \
                 -DBUILD_TESTS=OFF \
                 -DBUILD_PYTHON_BINDINGS=OFF \
                 -DBUILD_DOC=OFF
        make -j"$(nproc)"
        sudo make install && sudo ldconfig
    )
}

install_manif() {
    echo ""
    echo "=== manif (header-only, from source) ==="
    if [ -f /usr/local/include/manif/manif.h ]; then
        echo "manif already installed."
        return 0
    fi
    local src=/tmp/manif
    [ ! -d "$src" ] && git clone --depth 1 https://github.com/artivis/manif.git "$src"
    (
        cd "$src" && mkdir -p build && cd build
        cmake .. -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF -DBUILD_EXAMPLES=OFF
        sudo make install
    )
}

install_gtsam() {
    echo ""
    echo "=== GTSAM 4.2.0 (from source) ==="
    # borglab PPA does not publish for Ubuntu 24.04 (noble), so we source-build
    # on every host. Check for either the installed header or the shared lib.
    if [ -f /usr/local/include/gtsam/base/Matrix.h ] || ldconfig -p | grep -q libgtsam.so; then
        echo "GTSAM already installed."
        return 0
    fi
    local src=/tmp/gtsam
    [ ! -d "$src" ] && git clone --depth 1 --branch 4.2.0 https://github.com/borglab/gtsam.git "$src"
    (
        cd "$src" && mkdir -p build && cd build
        cmake .. -DCMAKE_BUILD_TYPE=Release \
                 -DGTSAM_BUILD_TESTS=OFF \
                 -DGTSAM_BUILD_EXAMPLES_ALWAYS=OFF \
                 -DGTSAM_BUILD_UNSTABLE=ON \
                 -DGTSAM_USE_SYSTEM_EIGEN=ON \
                 -DGTSAM_BUILD_WITH_MARCH_NATIVE=OFF
        make -j"$(nproc)"
        sudo make install && sudo ldconfig
    )
}

install_host_python() {
    echo ""
    echo "=== Host Python deps (workspace venv) ==="
    ensure_venv
    # Pinned via scripts/requirements-host.txt. numpy<2 keeps the venv aligned
    # with apt's numpy 1.26.4 (cv_bridge ABI). Skip opencv-python: use apt's
    # python3-opencv / cv_bridge's bundled cv2.
    pip install --upgrade -r "$_COMMON_SCRIPT_DIR/requirements-host.txt"
    pip uninstall -y opencv-python >/dev/null 2>&1 || true
}
