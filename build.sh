#!/bin/bash
# build.sh - Phased colcon build for perspective_grasp
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS_DIR="$(dirname "$(dirname "$SCRIPT_DIR")")"

cd "$WS_DIR"

CMAKE_ARGS="-DCMAKE_BUILD_TYPE=RelWithDebInfo"
PARALLEL_JOBS=$(( $(nproc) - 2 ))
[ "$PARALLEL_JOBS" -lt 1 ] && PARALLEL_JOBS=1

echo "=== Step 1/4: Building perception_msgs ==="
colcon build --packages-select perception_msgs \
    --cmake-args "$CMAKE_ARGS" \
    --parallel-workers "$PARALLEL_JOBS"

echo "=== Step 2/4: Building teaser_icp_hybrid_registrator ==="
colcon build --packages-select teaser_icp_hybrid_registrator \
    --cmake-args "$CMAKE_ARGS" \
    --parallel-workers "$PARALLEL_JOBS"

echo "=== Step 3/4: Building cross_camera_associator + pcl_merge_node ==="
colcon build --packages-select cross_camera_associator pcl_merge_node \
    --cmake-args "$CMAKE_ARGS" \
    --parallel-workers "$PARALLEL_JOBS"

echo "=== Step 4/4: Building remaining packages ==="
colcon build --packages-skip perception_msgs teaser_icp_hybrid_registrator \
    cross_camera_associator pcl_merge_node \
    --cmake-args "$CMAKE_ARGS" \
    --parallel-workers "$PARALLEL_JOBS"

echo "=== Build complete ==="
echo "Source with: source $WS_DIR/install/setup.bash"
