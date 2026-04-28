#!/bin/bash
# =============================================================================
# install_host.sh - Ubuntu 24.04 fresh-install path for perspective_grasp
#
# What this installs on the HOST:
#   - NVIDIA driver via `ubuntu-drivers autoinstall` (CUDA 12.6 Docker base
#     needs >=560; the autoinstall picks the recommended driver for the GPU,
#     which on noble today is one of 570/580/590-{open,server-open})
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
#   - Intel RealSense SDK + ROS 2 wrapper: opt-in, run scripts/install_realsense.sh
#     separately if you have a RealSense camera (D-series/L-series).
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

# Refuse to run as root. `sudo` is invoked per command; running the whole
# script under sudo poisons rosdep's cache (writes to /root/.ros) and bakes
# root ownership into the workspace venv + /tmp/{TEASER,manif,gtsam} clones,
# both of which fail user-mode rebuilds later with cryptic permission errors.
if [ "$EUID" -eq 0 ]; then
    echo "ERROR: do not run install_host.sh as root or via sudo." >&2
    echo "       Run as your normal user — sudo is invoked per command." >&2
    exit 1
fi

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_DIR="$(dirname "$SCRIPT_DIR")"

# shellcheck source=_deps_common.sh
source "$SCRIPT_DIR/_deps_common.sh"

echo "============================================================"
echo " perspective_grasp - Host Environment Setup"
echo " Target: Ubuntu 24.04 + NVIDIA GPU"
echo "============================================================"

# ---- 0. Enable universe + refresh apt index ----
# libpcl-dev, libceres-dev, libopenmpi-dev live in universe. (nvidia-driver-*
# is in `restricted`, which is enabled by default on Desktop installs but
# may not be on minimal/server — `ubuntu-drivers autoinstall` in step [1/8]
# will fail loudly if restricted is missing.) On a minimal install of 24.04
# universe is disabled by default and the rest of this script silently fails
# to find packages without this step.
echo ""
echo "=== [0/8] Enable apt universe ==="
sudo apt update
sudo apt install -y software-properties-common curl gnupg2 lsb-release ca-certificates
sudo add-apt-repository -y universe
sudo apt update

# ---- 1. NVIDIA Driver ----
echo ""
echo "=== [1/8] NVIDIA Driver (>=560) ==="
# Hardcoding `nvidia-driver-560` does not work on Ubuntu 24.04: noble's
# restricted component carries 535 / 570 / 580 / 590, and the exact set
# rotates as new branches reach stable. ubuntu-drivers picks the right
# variant (open vs proprietary, server vs desktop) for the installed GPU
# from whatever is currently published in restricted, which is what we
# actually want. CUDA 12.6 (Phase 4 Docker base) needs >=560 — every
# driver autoinstall picks today (570+) clears that bar.
if ! nvidia-smi &>/dev/null; then
    echo "Installing recommended NVIDIA driver via ubuntu-drivers autoinstall..."
    sudo apt install -y ubuntu-drivers-common
    sudo ubuntu-drivers autoinstall
    echo ""
    echo "** Reboot required after driver install. Run this script again after reboot. **"
    exit 0
else
    echo "NVIDIA driver already installed: $(nvidia-smi --query-gpu=driver_version --format=csv,noheader | head -1)"
fi

# ---- 2. ROS 2 Jazzy ----
echo ""
echo "=== [2/8] ROS 2 Jazzy ==="
if [ ! -f /opt/ros/jazzy/setup.bash ]; then
    # Use the canonical ros-apt-source .deb instead of hand-crafting
    # /etc/apt/sources.list.d/ros2.list with raw ros.key. The .deb tracks key
    # rotations on rosdistro automatically; the raw method silently breaks
    # with NO_PUBKEY when keys are rotated upstream.
    #
    # The version is pinned for reproducibility. Bump it when:
    #   - upstream rotates the signing key inside the .deb, or
    #   - the .deb stops resolving (release pulled or renamed).
    # GitHub does not redirect /releases/latest/download/<asset> for release
    # assets, so we cannot fetch "latest" without an extra `gh api` call —
    # an explicit pin is the lesser fragility.
    ROS_APT_SOURCE_VER="1.2.0"
    UBUNTU_CODENAME="$(. /etc/os-release && echo "$UBUNTU_CODENAME")"
    DEB_TMP="$(mktemp -d)"
    DEB_PATH="$DEB_TMP/ros2-apt-source.deb"
    curl -L -o "$DEB_PATH" \
        "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ROS_APT_SOURCE_VER}/ros2-apt-source_${ROS_APT_SOURCE_VER}.${UBUNTU_CODENAME}_all.deb"
    sudo apt install -y "$DEB_PATH"
    rm -rf "$DEB_TMP"
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
set +u
source /opt/ros/jazzy/setup.bash
set -u

# ---- 3. ROS 2 apt packages + system C++ libs ----
echo ""
echo "=== [3/8] ROS 2 packages + system C++ libs ==="
install_ros_apt_deps
sudo rosdep init 2>/dev/null || true
rosdep update --rosdistro=jazzy

# ---- 4. C++ libs from source (TEASER++, manif, GTSAM) ----
echo ""
echo "=== [4/8] C++ libs from source ==="
install_teaserpp
install_manif
install_gtsam

# ---- 5. Host Python deps (workspace venv) ----
echo ""
echo "=== [5/8] Host Python deps ==="
# torch wheel index is auto-selected from `nvidia-smi`'s "CUDA Version:" line
# (12.6→cu126, 12.8→cu128, 13.x→cu130, no GPU→cpu). Override only if you
# need to force a different build — see docs/installation.md.
#   PERSPECTIVE_TORCH_CUDA=cu128 ./scripts/install_host.sh    # explicit pin
#   PERSPECTIVE_TORCH_CUDA=cpu   ./scripts/install_host.sh    # headless / CI
install_host_python

# ---- 6. Docker + NVIDIA Container Toolkit ----
echo ""
echo "=== [6/8] Docker + nvidia-container-toolkit ==="
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
echo "=== [7/8] Model-weights directory scaffold ==="
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
echo "        $PROJECT_DIR/build.sh"
echo "   4. Build ML Docker containers:"
echo "        cd $PROJECT_DIR"
echo "        docker compose -f docker/docker-compose.yml build"
echo "   5. Drop weights under models/<service>/ (or set *_WEIGHTS env vars)."
echo "   6. (Optional) RealSense camera driver + ROS 2 wrapper:"
echo "        $PROJECT_DIR/scripts/install_realsense.sh"
echo ""
echo " Notes:"
echo "   - The torch wheel index was picked from nvidia-smi's CUDA Version."
echo "     If torch.cuda.is_available() still came out False above, force a"
echo "     specific build with PERSPECTIVE_TORCH_CUDA=cu126|cu128|cu130|cpu"
echo "     and re-run. See docs/installation.md."
echo "   - perception_system.launch.py auto-selects a host_profile (dev_8gb"
echo "     vs prod_16gb vs cpu_only) from VRAM. Override with"
echo "     host_profile:=<name> or PERSPECTIVE_HOST_PROFILE=<name>."
echo "============================================================"
