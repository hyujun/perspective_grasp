#!/bin/bash
# =============================================================================
# install_host.sh - Ubuntu 24.04 host environment setup for perspective_grasp
#
# What this installs (HOST):
#   - NVIDIA driver 560+ (required by CUDA 12.6 Docker base)
#   - CUDA toolkit (host-side; only needed if you build CUDA code on the host)
#   - ROS 2 Jazzy desktop
#   - C++ libraries (PCL, Eigen3, OpenCV, TEASER++, manif, GTSAM, Ceres, fmt)
#   - Python: ultralytics (YOLO)
#   - Docker + nvidia-container-toolkit (for Phase 4 ML nodes)
#
# What goes in Docker (see docker/docker-compose.yml):
#   - FoundationPose, CosyPose+MegaPose (shared), SAM2, BundleSDF
#   - Container stages ship their own CUDA 12.6 + PyTorch, so host CUDA
#     toolkit is NOT strictly required — only the NVIDIA driver is.
#
# Usage:
#   chmod +x scripts/install_host.sh
#   ./scripts/install_host.sh
#   # Reboot if NVIDIA driver was installed, then run again.
#   # Log out + back in after Docker install for group membership.
# =============================================================================
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_DIR="$(dirname "$SCRIPT_DIR")"

# shellcheck source=_venv.sh
source "$SCRIPT_DIR/_venv.sh"

echo "============================================================"
echo " perspective_grasp - Host Environment Setup"
echo " Target: Ubuntu 24.04 + NVIDIA GPU"
echo "============================================================"

# ---- 1. NVIDIA Driver ----
echo ""
echo "=== [1/9] NVIDIA Driver (560+) ==="
if ! nvidia-smi &>/dev/null; then
    echo "Installing NVIDIA driver 560..."
    sudo apt update
    sudo apt install -y nvidia-driver-560
    echo ""
    echo "** Reboot required after driver install. Run this script again after reboot. **"
    exit 0
else
    echo "NVIDIA driver already installed: $(nvidia-smi --query-gpu=driver_version --format=csv,noheader | head -1)"
fi

# ---- 2. CUDA Toolkit (host; optional — only for host-side CUDA compiles) ----
echo ""
echo "=== [2/9] CUDA Toolkit (host; optional) ==="
if ! nvcc --version &>/dev/null; then
    echo "Installing distro CUDA toolkit (nvidia-cuda-toolkit)..."
    sudo apt install -y nvidia-cuda-toolkit
    echo "CUDA installed (distro version)."
    echo "  Note: Phase 4 Docker stages bundle CUDA 12.6 independently."
else
    echo "CUDA already installed: $(nvcc --version | grep release)"
fi

# ---- 3. ROS 2 Jazzy ----
echo ""
echo "=== [3/9] ROS 2 Jazzy ==="
if [ ! -f /opt/ros/jazzy/setup.bash ]; then
    sudo apt install -y software-properties-common curl gnupg2 lsb-release
    sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
        -o /usr/share/keyrings/ros-archive-keyring.gpg
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" | \
        sudo tee /etc/apt/sources.list.d/ros2.list
    sudo apt update
    sudo apt install -y ros-jazzy-desktop
    # Add ROS 2 to bashrc if not already present
    if ! grep -q "source /opt/ros/jazzy/setup.bash" ~/.bashrc; then
        echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc
    fi
else
    echo "ROS 2 Jazzy already installed."
fi

# shellcheck disable=SC1091
source /opt/ros/jazzy/setup.bash

# ---- 4. ROS 2 Additional Packages + System Libraries ----
echo ""
echo "=== [4/9] ROS 2 Packages + System Libraries ==="
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

# Initialize rosdep
sudo rosdep init 2>/dev/null || true
rosdep update --rosdistro=jazzy

# ---- 5. C++ Libraries from Source (TEASER++, manif) ----
echo ""
echo "=== [5/9] TEASER++ + manif (from source) ==="

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
        sudo make install && sudo ldconfig
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

# ---- 6. GTSAM (enables HAS_GTSAM in pose_graph_smoother) ----
echo ""
echo "=== [6/9] GTSAM ==="
if ! dpkg -l | grep -q libgtsam-dev; then
    sudo add-apt-repository -y ppa:borglab/gtsam-release-4.2
    sudo apt update
    sudo apt install -y libgtsam-dev libgtsam-unstable-dev
else
    echo "GTSAM already installed."
fi

# ---- 7. Python Packages (host) ----
# Installed into the workspace venv ($WS_DIR/.venv) — NOT into ~/.local or
# system site-packages. This keeps pip-installed deps isolated from apt's
# python3-numpy / cv_bridge / rclpy, which stay visible via
# --system-site-packages.
echo ""
echo "=== [7/9] Python: ultralytics (YOLO) ==="
ensure_venv
# Pin numpy<2 so the venv keeps using apt's numpy 1.26.4 (what cv_bridge
# was built against). ultralytics pulls numpy as a dep and will otherwise
# install numpy>=2, reintroducing the C-extension ABI mismatch.
# Skip opencv-python: use apt's python3-opencv / cv_bridge's bundled cv2.
pip install --upgrade 'numpy<2' ultralytics
pip uninstall -y opencv-python >/dev/null 2>&1 || true

# ---- 8. Docker + NVIDIA Container Toolkit ----
echo ""
echo "=== [8/9] Docker + nvidia-container-toolkit ==="
if ! docker --version &>/dev/null; then
    echo "Installing Docker..."
    sudo apt install -y docker.io docker-compose-v2
    sudo usermod -aG docker "$USER"
    echo "Added $USER to docker group. Log out and back in to take effect."
fi

if ! dpkg -l | grep -q nvidia-container-toolkit; then
    echo "Installing nvidia-container-toolkit..."
    curl -fsSL https://nvidia.github.io/libnvidia-container/gpgkey | \
        sudo gpg --dearmor -o /usr/share/keyrings/nvidia-container-toolkit-keyring.gpg
    curl -s -L https://nvidia.github.io/libnvidia-container/stable/deb/nvidia-container-toolkit.list | \
        sed 's#deb https://#deb [signed-by=/usr/share/keyrings/nvidia-container-toolkit-keyring.gpg] https://#g' | \
        sudo tee /etc/apt/sources.list.d/nvidia-container-toolkit.list
    sudo apt update
    sudo apt install -y nvidia-container-toolkit
    sudo nvidia-ctk runtime configure --runtime=docker
    sudo systemctl restart docker
else
    echo "nvidia-container-toolkit already installed."
fi

# ---- 9. Model-weights directory scaffold ----
echo ""
echo "=== [9/9] Model-weights directory scaffold ==="
# Matches the default mount points in docker/docker-compose.yml.
# Override with env vars (FOUNDATIONPOSE_WEIGHTS, HAPPYPOSE_WEIGHTS,
# MEGAPOSE_MESHES, SAM2_WEIGHTS, BUNDLESDF_WEIGHTS) if weights live elsewhere.
mkdir -p \
    "$PROJECT_DIR/models/foundationpose" \
    "$PROJECT_DIR/models/happypose" \
    "$PROJECT_DIR/models/megapose/meshes" \
    "$PROJECT_DIR/models/sam2" \
    "$PROJECT_DIR/models/bundlesdf"
echo "Scaffold ready under: $PROJECT_DIR/models/"

echo ""
echo "============================================================"
echo " Host setup complete!"
echo ""
echo " Next steps:"
echo "   1. Log out and back in (for docker group)."
echo "   2. Activate the workspace Python venv (installed above):"
echo "        source $VENV_DIR/bin/activate"
echo "   3. Build the workspace:"
echo "        cd $WS_DIR"
echo "        ./src/perspective_grasp/build.sh"
echo "   4. Build ML Docker containers:"
echo "        cd $WS_DIR/src/perspective_grasp"
echo "        docker compose -f docker/docker-compose.yml build"
echo "   5. Drop weights under models/<service>/ (or set *_WEIGHTS env vars)."
echo "============================================================"
