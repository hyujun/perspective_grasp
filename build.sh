#!/bin/bash
# =============================================================================
# build.sh - Phased colcon build for perspective_grasp (18 packages)
#
# Build order respects inter-package dependencies:
#   1. perception_msgs + perception_launch_utils  (parallel; both are leaves of
#                                                  the dep graph and every other
#                                                  package needs them at build
#                                                  or launch time)
#   2. teaser_icp_hybrid_registrator        (C++ lib; yolo_pcl_cpp_tracker dep)
#   3. cross_camera_associator + pcl_merge_node  (multi-camera fusion infra)
#   4. All remaining packages               (parallel-safe)
#
# Usage:
#   ./build.sh                  # Full build (RelWithDebInfo)
#   ./build.sh Release          # Full build (Release)
#   ./build.sh --clean          # Wipe build/install/log then full build
#   ./build.sh --clean Release  # Clean + Release
#   ./build.sh --test           # Build then run unit tests
#   ./build.sh --help           # Show this help
#
# Flags (any order):
#   --clean   Remove build/install/log before building
#   --test    After build, run colcon test on packages that ship tests
#   Release | Debug | RelWithDebInfo | MinSizeRel   (build type; default RelWithDebInfo)
# =============================================================================
set -eo pipefail
# Note: `set -u` is intentionally omitted — colcon's generated setup.bash
# references COLCON_TRACE and other env vars without defaults, which would
# trip nounset every time we re-source after a phase.

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS_DIR="$(dirname "$(dirname "$SCRIPT_DIR")")"

# shellcheck source=scripts/_venv.sh
source "$SCRIPT_DIR/scripts/_venv.sh"

# ---- Parse arguments (flags in any order) ----
BUILD_TYPE="RelWithDebInfo"
DO_CLEAN=0
DO_TEST=0

usage() {
    sed -n '2,22p' "$0" | sed 's/^# \{0,1\}//'
    exit 0
}

for arg in "$@"; do
    case "$arg" in
        -h|--help) usage ;;
        --clean)   DO_CLEAN=1 ;;
        --test)    DO_TEST=1 ;;
        Release|Debug|RelWithDebInfo|MinSizeRel) BUILD_TYPE="$arg" ;;
        *) echo "Unknown argument: $arg (use --help)"; exit 1 ;;
    esac
done

cd "$WS_DIR"

CMAKE_ARGS="-DCMAKE_BUILD_TYPE=$BUILD_TYPE"
PARALLEL_JOBS=$(( $(nproc) - 2 ))
[ "$PARALLEL_JOBS" -lt 1 ] && PARALLEL_JOBS=1

# Ensure ROS 2 is sourced
if [ -z "${AMENT_PREFIX_PATH:-}" ]; then
    # shellcheck disable=SC1091
    source /opt/ros/jazzy/setup.bash
fi

# Activate the workspace venv (if one was created by install_*.sh). This
# makes ament_python packages stamp venv python into their script shebangs
# and ensures pip-installed runtime deps (e.g. ultralytics) are on sys.path
# when colcon builds + tests + when launched nodes spawn.
VENV_NOTE="(not present — Python deps fall back to system)"
if activate_venv_if_exists; then
    VENV_NOTE="$VIRTUAL_ENV"
fi

echo "============================================================"
echo " perspective_grasp - Workspace Build"
echo " Build type : $BUILD_TYPE"
echo " Parallel   : $PARALLEL_JOBS jobs"
echo " Workspace  : $WS_DIR"
echo " Python venv: $VENV_NOTE"
echo "============================================================"

if [ "$DO_CLEAN" = "1" ]; then
    echo ""
    echo "=== Cleaning build/install/log ==="
    rm -rf build install log
fi

# Helper: time a phase
phase() {
    local label="$1"; shift
    echo ""
    echo "=== $label ==="
    local t0=$SECONDS
    "$@"
    echo "    done in $((SECONDS - t0))s"
}

# ---- Phase 1: Leaf deps (every other package needs these at build or launch
#               time). perception_launch_utils is exec_depend-only today, so
#               colcon's topo sort doesn't force it before Phase 4 — we build
#               it here so partial `--packages-select` builds and launches
#               downstream don't trip on a missing helper. ----
phase "[1/4] perception_msgs + perception_launch_utils" \
    colcon build --packages-select perception_msgs perception_launch_utils \
        --cmake-args "$CMAKE_ARGS" \
        --parallel-workers "$PARALLEL_JOBS"
# shellcheck disable=SC1091
source install/setup.bash

# ---- Phase 2: Registration library (yolo_pcl_cpp_tracker depends on this) ----
phase "[2/4] teaser_icp_hybrid_registrator" \
    colcon build --packages-select teaser_icp_hybrid_registrator \
        --cmake-args "$CMAKE_ARGS" \
        --parallel-workers "$PARALLEL_JOBS"
# shellcheck disable=SC1091
source install/setup.bash

# ---- Phase 3: Multi-camera fusion infrastructure ----
phase "[3/4] cross_camera_associator + pcl_merge_node" \
    colcon build --packages-select cross_camera_associator pcl_merge_node \
        --cmake-args "$CMAKE_ARGS" \
        --parallel-workers "$PARALLEL_JOBS"
# shellcheck disable=SC1091
source install/setup.bash

# ---- Phase 4: All remaining packages (parallel-safe) ----
phase "[4/4] Remaining packages" \
    colcon build --packages-skip \
        perception_msgs \
        perception_launch_utils \
        teaser_icp_hybrid_registrator \
        cross_camera_associator \
        pcl_merge_node \
        --cmake-args "$CMAKE_ARGS" \
        --parallel-workers "$PARALLEL_JOBS"
# shellcheck disable=SC1091
source install/setup.bash

echo ""
echo "============================================================"
echo " Build complete ($BUILD_TYPE)"
echo " Source with: source $WS_DIR/install/setup.bash"
if [ -f "$VENV_DIR/bin/activate" ]; then
    echo " Venv   with: source $VENV_DIR/bin/activate"
fi
echo "============================================================"

# ---- Optional: run tests ----
if [ "$DO_TEST" = "1" ]; then
    echo ""
    echo "=== Running unit tests (Phase 1 + Phase 2 + Infrastructure) ==="
    colcon test --packages-select \
        teaser_icp_hybrid_registrator \
        yolo_pcl_cpp_tracker \
        cross_camera_associator \
        pcl_merge_node \
        multi_camera_calibration \
        perception_meta_controller \
        perception_debug_visualizer \
        perception_launch_utils
    colcon test-result --verbose || true
fi
