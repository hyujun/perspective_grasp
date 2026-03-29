#!/bin/bash
# =============================================================================
# build.sh - Phased colcon build for perspective_grasp
#
# Build order respects inter-package dependencies:
#   1. perception_msgs         (custom messages - all packages depend on this)
#   2. teaser_icp_hybrid_registrator (C++ library - yolo_pcl_cpp_tracker depends)
#   3. cross_camera_associator + pcl_merge_node (multi-camera infrastructure)
#   4. All remaining packages  (parallel safe)
#
# Usage:
#   ./build.sh              # Full build (RelWithDebInfo)
#   ./build.sh Release      # Full build (Release)
#   ./build.sh --clean      # Clean build directory first
# =============================================================================
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS_DIR="$(dirname "$(dirname "$SCRIPT_DIR")")"

cd "$WS_DIR"

# ---- Parse arguments ----
BUILD_TYPE="${1:-RelWithDebInfo}"
if [[ "${1:-}" == "--clean" ]]; then
    echo "=== Cleaning build/install/log directories ==="
    rm -rf build install log
    BUILD_TYPE="${2:-RelWithDebInfo}"
fi

CMAKE_ARGS="-DCMAKE_BUILD_TYPE=$BUILD_TYPE"
PARALLEL_JOBS=$(( $(nproc) - 2 ))
[ "$PARALLEL_JOBS" -lt 1 ] && PARALLEL_JOBS=1

# Ensure ROS 2 is sourced
if [ -z "${AMENT_PREFIX_PATH:-}" ]; then
    source /opt/ros/jazzy/setup.bash
fi

echo "============================================================"
echo " perspective_grasp - Workspace Build"
echo " Build type: $BUILD_TYPE | Parallel jobs: $PARALLEL_JOBS"
echo "============================================================"

# ---- Phase 1: Message definitions (all packages depend on this) ----
echo ""
echo "=== [1/4] perception_msgs ==="
colcon build --packages-select perception_msgs \
    --cmake-args "$CMAKE_ARGS" \
    --parallel-workers "$PARALLEL_JOBS"
source install/setup.bash

# ---- Phase 2: Registration library (yolo_pcl_cpp_tracker depends on this) ----
echo ""
echo "=== [2/4] teaser_icp_hybrid_registrator ==="
colcon build --packages-select teaser_icp_hybrid_registrator \
    --cmake-args "$CMAKE_ARGS" \
    --parallel-workers "$PARALLEL_JOBS"
source install/setup.bash

# ---- Phase 3: Multi-camera infrastructure ----
echo ""
echo "=== [3/4] cross_camera_associator + pcl_merge_node ==="
colcon build --packages-select cross_camera_associator pcl_merge_node \
    --cmake-args "$CMAKE_ARGS" \
    --parallel-workers "$PARALLEL_JOBS"
source install/setup.bash

# ---- Phase 4: All remaining packages (parallel safe) ----
echo ""
echo "=== [4/4] Remaining packages ==="
colcon build --packages-skip \
    perception_msgs \
    teaser_icp_hybrid_registrator \
    cross_camera_associator \
    pcl_merge_node \
    --cmake-args "$CMAKE_ARGS" \
    --parallel-workers "$PARALLEL_JOBS"

echo ""
echo "============================================================"
echo " Build complete!"
echo " Source with: source $WS_DIR/install/setup.bash"
echo "============================================================"
