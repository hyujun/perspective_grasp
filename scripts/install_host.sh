#!/bin/bash
# =============================================================================
# install_host.sh - Ubuntu 24.04 fresh-install path for perspective_grasp
#
# What this installs on the HOST:
#   - NVIDIA driver 560+ (required by CUDA 12.6 Docker base)
#   - ROS 2 Jazzy desktop
#   - C++ libraries via apt (PCL, Eigen3, OpenCV, Ceres, fmt, openmpi)
#   - C++ libraries from source (TEASER++, manif, GTSAM 4.2.0)
#   - Python host deps in workspace venv (ultralytics + numpy<2)
#   - Docker + nvidia-container-toolkit (for Phase 4 ML nodes)
#
# What this DOES NOT install (by design):
#   - Host CUDA toolkit (nvcc): all ML nodes run in Docker containers that
#     bundle their own CUDA 12.6. The NVIDIA driver alone is sufficient.
#   - Phase 4 ML Python stacks (PyTorch, kaolin, etc.): those live in
#     docker/Dockerfile stages. See docker/docker-compose.yml.
#
# For a host that already has ROS 2 Jazzy, use install_dependencies.sh
# instead — it skips driver/ROS install and runs only the additional deps.
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

# shellcheck source=_deps_common.sh
source "$SCRIPT_DIR/_deps_common.sh"

echo "============================================================"
echo " perspective_grasp - Host Environment Setup"
echo " Target: Ubuntu 24.04 + NVIDIA GPU"
echo "============================================================"

# ---- 1. NVIDIA Driver ----
echo ""
echo "=== [1/7] NVIDIA Driver (560+) ==="
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

# ---- 2. ROS 2 Jazzy ----
echo ""
echo "=== [2/7] ROS 2 Jazzy ==="
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

# ---- 3. ROS 2 apt packages + system C++ libs ----
echo ""
echo "=== [3/7] ROS 2 packages + system C++ libs ==="
install_ros_apt_deps
sudo rosdep init 2>/dev/null || true
rosdep update --rosdistro=jazzy

# ---- 4. C++ libs from source (TEASER++, manif, GTSAM) ----
echo ""
echo "=== [4/7] C++ libs from source ==="
install_teaserpp
install_manif
install_gtsam

# ---- 5. Host Python deps (workspace venv) ----
echo ""
echo "=== [5/7] Host Python deps ==="
install_host_python

# ---- 6. Docker + NVIDIA Container Toolkit ----
echo ""
echo "=== [6/7] Docker + nvidia-container-toolkit ==="
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

# ---- 7. Model-weights directory scaffold ----
echo ""
echo "=== [7/7] Model-weights directory scaffold ==="
# Matches the default mount points in docker/docker-compose.yml. Override with
# env vars (FOUNDATIONPOSE_WEIGHTS, HAPPYPOSE_WEIGHTS, MEGAPOSE_MESHES,
# SAM2_WEIGHTS, BUNDLESDF_WEIGHTS) if weights live elsewhere.
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
