# perspective_grasp

RGB-D camera-based 6D pose estimation pipeline with UR5e + 10-DoF hand manipulation system. 16 ROS 2 packages across 5 pipeline phases, supporting 1-3 cameras with config-driven topology.

## Architecture

**Vision Push, Controller Pull** — Vision system broadcasts TF2 transforms continuously; the robot controller does `lookupTransform()` at its own rate. Heavy operations (scene analysis, grasp planning) use ROS 2 Action Servers to avoid blocking the control loop.

```
                         ┌──────────────────────────────────────────┐
                         │          Camera Drivers (1-3)            │
                         │     RGB + Depth + CameraInfo per cam     │
                         └──────────┬───────────────────────────────┘
                                    │
                    ┌───────────────▼───────────────┐
                    │     Phase 1: Detection &      │
                    │     Pose Estimation            │
                    │  ┌─────────────────────────┐   │
                    │  │  yolo_pcl_cpp_tracker    │   │
                    │  │  (YOLO + ByteTrack +     │   │
                    │  │   TEASER++ / ICP)        │   │
                    │  └────────────┬─────────────┘   │
                    └───────────────┼──────────────────┘
                                    │ per-camera raw poses
               ┌────────────────────▼────────────────────┐
               │  Phase 2: Multi-Camera Fusion           │
               │  ┌────────────────┐  ┌───────────────┐  │
               │  │ cross_camera_  │  │ pcl_merge_    │  │
               │  │ associator     │  │ node          │  │
               │  │ (Hungarian +   │  │ (point cloud  │  │
               │  │  Union-Find)   │  │  merging)     │  │
               │  └───────┬────────┘  └───────────────┘  │
               └──────────┼──────────────────────────────┘
                          │ associated poses (global IDs)
               ┌──────────▼──────────────────────────────┐
               │  Phase 3: Filtering & Smoothing         │
               │  ┌────────────────┐  ┌───────────────┐  │
               │  │ pose_filter_   │  │ pose_graph_   │  │
               │  │ cpp            │  │ smoother      │  │
               │  │ (SE(3) IEKF)   │  │ (GTSAM opt.)  │  │
               │  └───────┬────────┘  └───────┬───────┘  │
               └──────────┼───────────────────┼──────────┘
                          │                   │
               ┌──────────▼───────────────────▼──────────┐
               │  Phase 4: Refinement (On-Demand)        │
               │  ┌──────────────┐  ┌─────────────────┐  │
               │  │ foundationpose│  │ cosypose_scene_ │  │
               │  │ _tracker     │  │ optimizer       │  │
               │  ├──────────────┤  ├─────────────────┤  │
               │  │ megapose_    │  │ sam2_instance_  │  │
               │  │ ros2_wrapper │  │ segmentor       │  │
               │  ├──────────────┤  └─────────────────┘  │
               │  │ bundlesdf_   │                       │
               │  │ unknown_     │                       │
               │  │ tracker      │                       │
               │  └──────────────┘                       │
               └─────────────────────────────────────────┘
                          │
               ┌──────────▼──────────────────────────────┐
               │  Phase 5: Grasp Planning                │
               │  ┌──────────────────────────────────┐   │
               │  │ grasp_pose_planner               │   │
               │  │ (Action Server → UR5e + 10-DoF)  │   │
               │  └──────────────────────────────────┘   │
               └─────────────────────────────────────────┘

     Orchestration: perception_meta_controller (NORMAL / HIGH_PRECISION / SCENE_ANALYSIS)
     Debug:         perception_debug_visualizer (OpenCV overlay)
     Calibration:   multi_camera_calibration (ChArUco + hand-eye + Ceres)
```

## Packages

### Messages & Interfaces

| Package | Description |
|---------|-------------|
| [perception_msgs](perception_msgs/) | Custom message, service, and action definitions (11 msgs, 1 srv, 2 actions) |

### Phase 1: Detection & Pose Estimation

| Package | Language | Description |
|---------|----------|-------------|
| [yolo_pcl_cpp_tracker](yolo_pcl_cpp_tracker/) | C++ / Python | YOLO + ByteTrack detection with ICP-based 6D pose estimation |
| [teaser_icp_hybrid_registrator](teaser_icp_hybrid_registrator/) | C++ | Library: TEASER++ global registration + ICP local refinement |

### Phase 2: Multi-Camera Fusion

| Package | Language | Description |
|---------|----------|-------------|
| [cross_camera_associator](cross_camera_associator/) | C++ | Hungarian algorithm + Union-Find for cross-camera object matching |
| [pcl_merge_node](pcl_merge_node/) | C++ | Point cloud merging, filtering, deduplication from multiple cameras |

### Phase 3: Filtering & Smoothing

| Package | Language | Description |
|---------|----------|-------------|
| [pose_filter_cpp](pose_filter_cpp/) | C++ | SE(3) IEKF for multi-source pose fusion with outlier rejection |
| [pose_graph_smoother](pose_graph_smoother/) | C++ | Sliding-window pose graph optimization (GTSAM, optional) |

### Phase 4: Refinement (On-Demand)

| Package | Language | Description |
|---------|----------|-------------|
| [isaac_foundationpose_tracker](isaac_foundationpose_tracker/) | Python | FoundationPose zero-shot 6D pose tracker (stub) |
| [megapose_ros2_wrapper](megapose_ros2_wrapper/) | Python | MegaPose zero-shot 6D pose estimation (stub) |
| [bundlesdf_unknown_tracker](bundlesdf_unknown_tracker/) | Python | BundleSDF neural implicit surface tracking for unknown objects (stub) |
| [sam2_instance_segmentor](sam2_instance_segmentor/) | Python | SAM2 instance segmentation (stub) |
| [cosypose_scene_optimizer](cosypose_scene_optimizer/) | Python | CosyPose scene-level joint pose optimization (stub) |

### Phase 5: Grasp Planning

| Package | Language | Description |
|---------|----------|-------------|
| [grasp_pose_planner](grasp_pose_planner/) | Python | Grasp pose action server for UR5e + 10-DoF hand (stub) |

### Infrastructure

| Package | Language | Description |
|---------|----------|-------------|
| [perception_meta_controller](perception_meta_controller/) | C++ | Pipeline mode orchestrator (NORMAL / HIGH_PRECISION / SCENE_ANALYSIS) |
| [perception_debug_visualizer](perception_debug_visualizer/) | C++ | Real-time OpenCV debug overlay |
| [multi_camera_calibration](multi_camera_calibration/) | C++ / Python | Hand-eye calibration with ChArUco + optional Ceres joint optimization |

## Pipeline Modes

| Mode | Description | Active Nodes |
|------|-------------|-------------|
| **NORMAL** | Real-time tracking | YOLO tracker + ICP + pose filter |
| **HIGH_PRECISION** | High-accuracy manipulation | NORMAL + FoundationPose + pose graph smoother |
| **SCENE_ANALYSIS** | Scene understanding | YOLO tracker + SAM2 + CosyPose + pose filter |

## Multi-Camera Support

1-3 cameras via config-driven topology. Single YAML defines camera count and type.

- **Config**: `config/camera_config.yaml` (3-cam), `camera_config_1cam.yaml`, `camera_config_2cam.yaml`
- **Per-camera**: Nodes namespaced as `/cam0/`, `/cam1/`, `/cam2/`
- **Fusion**: `cross_camera_associator` matches objects across cameras
- **Point cloud merge**: `pcl_merge_node` merges eye-to-hand depth clouds
- **Calibration**: `multi_camera_calibration` with ChArUco + optional Ceres joint optimization

## Build

```bash
cd /home/junho/ros2_ws/perspective_ws

# Full build (respects dependency order)
./src/perspective_grasp/build.sh

# Single package
colcon build --packages-select <package_name>

# Source after build
source install/setup.bash
```

### Build Order

1. `perception_msgs` (all packages depend on this)
2. `teaser_icp_hybrid_registrator` (library)
3. `cross_camera_associator` + `pcl_merge_node` (multi-camera infrastructure)
4. All remaining packages (parallel safe)

## Key Topics

### Per-Camera (Namespaced)

| Topic | Type | Description |
|-------|------|-------------|
| `/{ns}/yolo/detections` | `DetectionArray` | 2D detections |
| `/{ns}/yolo_tracker/raw_poses` | `PoseWithMetaArray` | Raw 6D poses |

### Cross-Camera Fusion

| Topic | Type | Description |
|-------|------|-------------|
| `/associated/poses` | `AssociatedPoseArray` | Associated poses with global IDs |
| `/merged/points` | `PointCloud2` | Merged point cloud |

### Downstream (Global)

| Topic | Type | Description |
|-------|------|-------------|
| `/pose_filter/filtered_poses` | `PoseWithMetaArray` | IEKF-filtered poses |
| `/smoother/smoothed_poses` | `PoseWithMetaArray` | Graph-smoothed poses |
| `/foundationpose/raw_poses` | `PoseWithMetaArray` | FoundationPose output |
| `/cosypose/optimized_poses` | `PoseWithMetaArray` | Scene-optimized poses |
| `/sam2/masks` | `SegmentationArray` | Instance segmentation masks |
| `/meta_controller/active_pipeline` | `PipelineStatus` | Pipeline status |

## TF2 Frame Convention

### Single Camera

- Parent: `camera_color_optical_frame`
- Child: `object_{track_id}_filtered`

### Multi-Camera

- Per-camera frames: `cam{N}_color_optical_frame`
- All filtered poses in `ur5e_base_link` frame
- Static TF: `ur5e_base_link` -> `cam{N}_link` (from calibration)

## Code Conventions

- **C++ Standard**: C++20
- **Compiler Flags**: `-Wall -Wextra -Wpedantic -Wshadow -Wconversion`
- **Formatting**: clang-format (Google style with modifications)
- **Namespace**: `perspective_grasp`

## Key Constraints

- **GPU**: RTX 3070 Ti (8GB VRAM) — only YOLO + one GPU-heavy node at a time
- **QoS**: BEST_EFFORT + depth 1 for control-path topics
- **Optional dependencies**: TEASER++ (ICP fallback), Ceres (OpenCV fallback), GTSAM (passthrough)

## License

Apache-2.0
