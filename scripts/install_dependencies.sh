#!/bin/bash
# =============================================================================
# install_dependencies.sh - Install additional deps when ROS 2 already exists.
#
# Use this when ROS 2 Jazzy is already installed and you only need the
# additional C++/Python libraries. For a full Ubuntu 24.04 setup from
# scratch (incl. NVIDIA driver + ROS 2 + Docker), use install_host.sh instead.
#
# External dependencies by package:
#   perception_msgs                : (pure msgs, no externals)
#   teaser_icp_hybrid_registrator  : Eigen3, PCL, TEASER++ (HAS_TEASERPP)
#   yolo_pcl_cpp_tracker           : PCL, Eigen3, ultralytics (venv)
#   cross_camera_associator        : Eigen3
#   pcl_merge_node                 : PCL, MPI
#   pose_filter_cpp                : Eigen3, manif
#   pose_graph_smoother            : Eigen3, GTSAM (HAS_GTSAM)
#   multi_camera_calibration       : OpenCV, Eigen3, Ceres (HAS_CERES)
#   perception_debug_visualizer    : OpenCV, image_transport, cv_bridge
#   perception_meta_controller     : (ROS deps only)
#   perception_bringup             : (launch-only)
#   grasp_pose_planner             : sensor_msgs_py, rclpy
#   Phase 4 ML nodes               : packaged in Docker (docker/Dockerfile)
# =============================================================================
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# shellcheck source=_deps_common.sh
source "$SCRIPT_DIR/_deps_common.sh"

echo "============================================================"
echo " perspective_grasp - Dependency Installation"
echo "============================================================"

install_ros_apt_deps
install_teaserpp
install_manif
install_gtsam
install_host_python

echo ""
echo "============================================================"
echo " All host-side dependencies installed successfully!"
echo ""
echo " Python venv: $VENV_DIR"
echo "   Activate before running nodes:  source $VENV_DIR/bin/activate"
echo ""
echo " Phase 4 ML nodes (FoundationPose, CosyPose, MegaPose, SAM2,"
echo " BundleSDF) run inside Docker — build those with:"
echo "   cd $WS_DIR/src/perspective_grasp"
echo "   docker compose -f docker/docker-compose.yml build"
echo ""
echo " Next: cd $WS_DIR && ./src/perspective_grasp/build.sh"
echo "============================================================"
