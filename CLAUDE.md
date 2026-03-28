# perspective_grasp - 6D Pose Estimation & Manipulation System

## Project Overview
RGB-D camera-based 6D pose estimation pipeline + UR5e + 10-DoF hand manipulation.
16 ROS 2 packages across 5 phases + debug visualizer + multi-camera support.

## Architecture Principle
**Vision Push, Controller Pull.** Vision broadcasts TF2 continuously; Controller does `lookupTransform()` at its own rate.
Heavy ops (Scene analysis, Grasp planning) use ROS 2 Action Servers to avoid blocking the control loop.

## Multi-Camera Support
1~3 cameras supported via config-driven topology. Single YAML defines camera count/type.

- **Config**: `config/camera_config.yaml` (3-cam), `camera_config_1cam.yaml`, `camera_config_2cam.yaml`
- **Principle**: N=1 treated as "N=1 multi-camera" (unified code path, no special case)
- **Per-camera**: Nodes namespaced as `/cam0/`, `/cam1/`, `/cam2/`
- **Fusion**: `cross_camera_associator` matches objects across cameras (Hungarian + Union-Find)
- **Point cloud merge**: `pcl_merge_node` merges eye-to-hand depth clouds
- **Calibration**: `multi_camera_calibration` with ChArUco + optional Ceres joint optimization

## Build

```bash
# Full build (respects dependency order)
cd /home/junho/ros2_ws/perspective_ws
./src/perspective_grasp/build.sh

# Single package
colcon build --packages-select <package_name>

# Source after build
source install/setup.bash
```

## Build Order
1. `perception_msgs` (all packages depend on this)
2. `teaser_icp_hybrid_registrator` (library, yolo_pcl_cpp_tracker depends on this)
3. `cross_camera_associator` + `pcl_merge_node` (multi-camera infrastructure)
4. All remaining packages (parallel safe)

## Code Conventions
- **C++ Standard**: C++20 (`cxx_std_20`)
- **Compiler flags**: `-Wall -Wextra -Wpedantic -Wshadow -Wconversion`
- **C++ build**: `ament_cmake`
- **Python build**: `ament_cmake_python` (for mixed packages) or `ament_python`
- **Formatting**: clang-format (Google style with modifications)
- **Namespace**: `perspective_grasp`

## Topic Naming

### Per-camera (namespaced)
- `/{ns}/yolo/detections` (DetectionArray)
- `/{ns}/yolo_tracker/raw_poses` (PoseWithMetaArray)

### Cross-camera fusion
- `/associated/poses` (AssociatedPoseArray) — from cross_camera_associator
- `/merged/points` (PointCloud2) — from pcl_merge_node

### Downstream (global)
- `/pose_filter/filtered_poses` (PoseWithMetaArray)
- `/smoother/smoothed_poses` (PoseWithMetaArray)
- `/foundationpose/raw_poses` (PoseWithMetaArray)
- `/sam2/masks` (SegmentationArray)
- `/cosypose/optimized_poses` (PoseWithMetaArray)
- `/meta_controller/active_pipeline` (PipelineStatus)

## TF2 Frame Convention

### Single camera
- Parent: `camera_color_optical_frame`
- Child: `object_{track_id}_filtered`

### Multi-camera
- Per-camera frames: `cam0_color_optical_frame`, `cam1_color_optical_frame`, `cam2_color_optical_frame`
- All filtered poses in `ur5e_base_link` frame
- Static TF: `ur5e_base_link` → `cam{N}_link` (from calibration)
- Per-camera: `cam{N}_link` → `cam{N}_color_optical_frame` (from camera driver)

## Key Constraints
- GPU: RTX 3070 Ti (8GB VRAM) - only YOLO + one GPU-heavy node at a time
- No cross-workspace code dependency with ur5e_ws (communicate via TF2 and actions only)
- QoS: BEST_EFFORT + depth 1 for control-path topics
- Optional dependencies: TEASER++ (ICP fallback), Ceres (OpenCV fallback), GTSAM (passthrough)

## Workspace Paths
- This repo: `/home/junho/ros2_ws/perspective_ws/src/perspective_grasp/`
- Controller: `/home/junho/ros2_ws/ur5e_ws/src/ur5e-rt-controller/`
- ROS 2: `/opt/ros/jazzy/`
