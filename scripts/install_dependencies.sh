#!/bin/bash
# install_dependencies.sh - Install all external dependencies for perspective_grasp
set -euo pipefail

echo "=== Installing system packages ==="
sudo apt update
sudo apt install -y \
    ros-jazzy-tf2-ros \
    ros-jazzy-tf2-eigen \
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
    libopencv-dev

echo "=== Installing TEASER++ (from source) ==="
TEASER_DIR=/tmp/TEASER-plusplus
if [ ! -d "$TEASER_DIR" ]; then
    git clone https://github.com/MIT-SPARK/TEASER-plusplus.git "$TEASER_DIR"
fi
cd "$TEASER_DIR"
mkdir -p build && cd build
cmake .. -DCMAKE_BUILD_TYPE=Release \
         -DBUILD_TESTS=OFF \
         -DBUILD_PYTHON_BINDINGS=OFF \
         -DBUILD_DOC=OFF
make -j"$(nproc)"
sudo make install
sudo ldconfig

echo "=== Installing manif (header-only SE(3) library) ==="
MANIF_DIR=/tmp/manif
if [ ! -d "$MANIF_DIR" ]; then
    git clone https://github.com/artivis/manif.git "$MANIF_DIR"
fi
cd "$MANIF_DIR"
mkdir -p build && cd build
cmake .. -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF -DBUILD_EXAMPLES=OFF
sudo make install

echo "=== Installing GTSAM ==="
sudo add-apt-repository -y ppa:borglab/gtsam-release-4.2
sudo apt update
sudo apt install -y libgtsam-dev libgtsam-unstable-dev

echo "=== Installing Python ML packages ==="
pip3 install --user ultralytics

echo "=== All dependencies installed successfully ==="
