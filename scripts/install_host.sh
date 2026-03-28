#!/bin/bash
# =============================================================================
# install_host.sh - Ubuntu 24.04 host environment setup for perspective_grasp
#
# What this installs (HOST):
#   - NVIDIA driver + CUDA toolkit
#   - ROS 2 Jazzy desktop
#   - C++ libraries (PCL, Eigen3, OpenCV, TEASER++, manif, GTSAM, Ceres)
#   - YOLO (ultralytics)
#   - Docker + nvidia-container-toolkit (for ML nodes)
#
# What goes in Docker (see docker-compose.yml):
#   - FoundationPose, MegaPose/CosyPose, SAM2, BundleSDF
# =============================================================================
set -euo pipefail

echo "============================================================"
echo " perspective_grasp - Host Environment Setup"
echo " Target: Ubuntu 24.04 + NVIDIA GPU"
echo "============================================================"

# ---- 1. NVIDIA Driver ----
echo ""
echo "=== [1/8] NVIDIA Driver ==="
if ! nvidia-smi &>/dev/null; then
    echo "Installing NVIDIA driver..."
    sudo apt update
    sudo apt install -y nvidia-driver-560
    echo "⚠  Reboot required after driver install. Run this script again after reboot."
    exit 0
else
    echo "NVIDIA driver already installed: $(nvidia-smi --query-gpu=driver_version --format=csv,noheader | head -1)"
fi

# ---- 2. CUDA Toolkit ----
echo ""
echo "=== [2/8] CUDA Toolkit 12.4 ==="
if ! nvcc --version &>/dev/null; then
    echo "Installing CUDA toolkit..."
    sudo apt install -y nvidia-cuda-toolkit
    echo "CUDA installed."
else
    echo "CUDA already installed: $(nvcc --version | grep release)"
fi

# ---- 3. ROS 2 Jazzy ----
echo ""
echo "=== [3/8] ROS 2 Jazzy ==="
if [ ! -f /opt/ros/jazzy/setup.bash ]; then
    sudo apt install -y software-properties-common curl gnupg2 lsb-release
    sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
        -o /usr/share/keyrings/ros-archive-keyring.gpg
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
        http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" | \
        sudo tee /etc/apt/sources.list.d/ros2.list
    sudo apt update
    sudo apt install -y ros-jazzy-desktop
    echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc
else
    echo "ROS 2 Jazzy already installed."
fi

source /opt/ros/jazzy/setup.bash

# ---- 4. ROS 2 Additional Packages ----
echo ""
echo "=== [4/8] ROS 2 Packages + System Libraries ==="
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
    libfmt-dev \
    libpcl-dev \
    libeigen3-dev \
    libopencv-dev \
    libopenmpi-dev \
    libceres-dev

sudo rosdep init 2>/dev/null || true
rosdep update --rosdistro=jazzy

# ---- 5. C++ Libraries from Source ----
echo ""
echo "=== [5/8] TEASER++ + manif (from source) ==="

# TEASER++
TEASER_DIR=/tmp/TEASER-plusplus
if ! ldconfig -p | grep -q teaserpp; then
    echo "Building TEASER++..."
    [ ! -d "$TEASER_DIR" ] && git clone --depth 1 https://github.com/MIT-SPARK/TEASER-plusplus.git "$TEASER_DIR"
    cd "$TEASER_DIR" && mkdir -p build && cd build
    cmake .. -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTS=OFF -DBUILD_PYTHON_BINDINGS=OFF -DBUILD_DOC=OFF
    make -j"$(nproc)"
    sudo make install && sudo ldconfig
else
    echo "TEASER++ already installed."
fi

# manif
MANIF_DIR=/tmp/manif
if [ ! -f /usr/local/include/manif/manif.h ]; then
    echo "Installing manif..."
    [ ! -d "$MANIF_DIR" ] && git clone --depth 1 https://github.com/artivis/manif.git "$MANIF_DIR"
    cd "$MANIF_DIR" && mkdir -p build && cd build
    cmake .. -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF -DBUILD_EXAMPLES=OFF
    sudo make install
else
    echo "manif already installed."
fi

# ---- 6. GTSAM ----
echo ""
echo "=== [6/8] GTSAM ==="
if ! dpkg -l | grep -q libgtsam-dev; then
    sudo add-apt-repository -y ppa:borglab/gtsam-release-4.2
    sudo apt update
    sudo apt install -y libgtsam-dev libgtsam-unstable-dev
else
    echo "GTSAM already installed."
fi

# ---- 7. Python Packages (host) ----
echo ""
echo "=== [7/8] Python: ultralytics (YOLO) ==="
pip3 install --user ultralytics

# ---- 8. Docker + NVIDIA Container Toolkit ----
echo ""
echo "=== [8/8] Docker + nvidia-container-toolkit ==="
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

echo ""
echo "============================================================"
echo " Host setup complete!"
echo ""
echo " Next steps:"
echo "   1. Log out and back in (for docker group)"
echo "   2. Build the workspace:"
echo "        cd ~/ros2_ws/perspective_ws"
echo "        ./src/perspective_grasp/build.sh"
echo "   3. Build ML Docker containers:"
echo "        cd ~/ros2_ws/perspective_ws/src/perspective_grasp"
echo "        docker compose build"
echo "============================================================"
