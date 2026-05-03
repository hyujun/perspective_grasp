# Architecture Reference

Topic names, TF frames, pipeline modes, and the design principles behind the layout.

## Design principles

- **Vision Push, Controller Pull.** The vision stack broadcasts TF2 transforms continuously. The robot controller calls `lookupTransform()` at its own rate. The controller never waits on a vision topic callback.
- **Heavy ops are Action Servers.** Scene analysis and grasp planning are long-running operations — they are exposed as ROS 2 actions, not services, so the control loop never blocks on them.
- **Host + Docker hybrid.** C++ perception nodes and YOLO run on the host. GPU-heavy Python ML nodes (FoundationPose, SAM2, CosyPose, MegaPose, BundleSDF) run in containers for dependency isolation.
- **Unified multi-camera code path.** `N=1` is treated as "N=1 multi-camera". Configuration lives in YAML; launch files and fusion code never branch on camera count.
- **No cross-workspace code coupling.** The UR5e controller workspace communicates with this repo only via TF2 frames and action interfaces — never via imported types or shared libraries.

## Pipeline overview

```
                     ┌──────────────────────────────────────────┐
                     │          Camera Drivers (1-3)            │
                     │     RGB + Depth + CameraInfo per cam     │
                     └──────────┬───────────────────────────────┘
                                │
                ┌───────────────▼───────────────┐
                │  Phase 1: Detection &         │
                │         Pose Estimation       │
                │  yolo_pcl_cpp_tracker         │
                │  (YOLO + ByteTrack +          │
                │   TEASER++ / ICP)             │
                └───────────────┬───────────────┘
                                │ per-camera raw poses
           ┌────────────────────▼────────────────────┐
           │  Phase 2: Multi-Camera Fusion           │
           │  cross_camera_associator  pcl_merge     │
           │  (Hungarian + Union-Find)  (cloud merge)│
           └────────────────────┬────────────────────┘
                                │ associated poses (global IDs)
           ┌────────────────────▼────────────────────┐
           │  Phase 3: Filtering & Smoothing         │
           │  pose_filter_cpp   pose_graph_smoother  │
           │  (SE(3) IEKF)      (GTSAM, optional)    │
           └────────────────────┬────────────────────┘
                                │
           ┌────────────────────▼────────────────────┐
           │  Phase 4: Refinement (On-Demand)        │
           │  foundationpose  sam2  cosypose         │
           │  megapose        bundlesdf              │
           │  (all shipping; SAM2 live-verified)     │
           └────────────────────┬────────────────────┘
                                │
           ┌────────────────────▼────────────────────┐
           │  Phase 5: Grasp Planning                │
           │  grasp_pose_planner (Action Server)     │
           └─────────────────────────────────────────┘

  Orchestration: perception_meta_controller  (NORMAL / HIGH_PRECISION / SCENE_ANALYSIS)
  Debug:         perception_debug_visualizer (OpenCV overlay)
  Calibration:   multi_camera_calibration    (ChArUco + hand-eye + Ceres)
```

## Packages (18 total)

| Group | Package | Language |
|-------|---------|----------|
| Interfaces | [perception_msgs](../packages/interfaces/perception_msgs/) | msg/srv/action |
| Bringup | [perception_bringup](../packages/bringup/perception_bringup/) | launch + config |
| Phase 1 | [yolo_pcl_cpp_tracker](../packages/phase1_perception/yolo_pcl_cpp_tracker/) | C++ / Python |
| Phase 1 | [teaser_icp_hybrid_registrator](../packages/phase1_perception/teaser_icp_hybrid_registrator/) | C++ |
| Phase 2 | [cross_camera_associator](../packages/phase2_fusion/cross_camera_associator/) | C++ |
| Phase 2 | [pcl_merge_node](../packages/phase2_fusion/pcl_merge_node/) | C++ |
| Phase 3 | [pose_filter_cpp](../packages/phase3_filtering/pose_filter_cpp/) | C++ |
| Phase 3 | [pose_graph_smoother](../packages/phase3_filtering/pose_graph_smoother/) | C++ |
| Phase 4 | [isaac_foundationpose_tracker](../packages/phase4_refinement/isaac_foundationpose_tracker/) | Python |
| Phase 4 | [megapose_ros2_wrapper](../packages/phase4_refinement/megapose_ros2_wrapper/) | Python |
| Phase 4 | [cosypose_scene_optimizer](../packages/phase4_refinement/cosypose_scene_optimizer/) | Python |
| Phase 4 | [sam2_instance_segmentor](../packages/phase4_refinement/sam2_instance_segmentor/) | Python |
| Phase 4 | [bundlesdf_unknown_tracker](../packages/phase4_refinement/bundlesdf_unknown_tracker/) | Python |
| Phase 5 | [grasp_pose_planner](../packages/phase5_manipulation/grasp_pose_planner/) | Python |
| Infra | [perception_meta_controller](../packages/infrastructure/perception_meta_controller/) | C++ |
| Infra | [perception_debug_visualizer](../packages/infrastructure/perception_debug_visualizer/) | C++ |
| Infra | [multi_camera_calibration](../packages/infrastructure/multi_camera_calibration/) | C++ / Python |
| Infra | [perception_launch_utils](../packages/infrastructure/perception_launch_utils/) | Python |

### Launch helpers — `perception_launch_utils`

Every `launch.py` in this workspace imports from `perception_launch_utils` for these concerns:

| Helper | Replaces / does |
|--------|-----------------|
| `config_path(pkg, filename=None)` | `os.path.join(get_package_share_directory(pkg), 'config', filename)` |
| `share_file(pkg, *parts)` | The same pattern for non-`config/` resources (rviz, meshes...) |
| `declare_params_file_arg`, `declare_camera_config_arg`, `declare_host_profile_arg`, `declare_autostart_arg` | The boilerplate `DeclareLaunchArgument` stanzas |
| `load_config(path)` → `PerceptionSystemConfig` | The old `importlib`-based `camera_config_loader.py` shim from `perception_bringup` |
| `fanout_lifecycle_nodes(...)` | The ~140-line `OpaqueFunction + yaml.safe_load + per-camera LifecycleNode + autostart event wiring` block that used to be copy-pasted across the five Phase 4 launch files |
| `resolve_torch_device(requested, logger)` → `DeviceResolution` | Picks a verified torch device string for GPU-using nodes, falling back to CPU with one WARN when the cuXXX wheel can't drive the deployed NVIDIA driver |
| `resolve_host_profile(name)` → `HostProfile` + `overrides_for_node(profile, name)` | Loads `dev_8gb`/`prod_16gb`/`cpu_only` parameter overrides from `packages/infrastructure/perception_launch_utils/host_profiles/` (installed to `share/perception_launch_utils/host_profiles/`). `auto` picks by total VRAM |
| `preflight_launch_action(strict=...)` | One-shot `OpaqueFunction` that prints driver/torch versions at launch time and warns on mismatch |

See [perception_launch_utils/README.md](../packages/infrastructure/perception_launch_utils/README.md) for the full API and migration examples.

### Host profiles & dev↔execution PC split

The dev box (RTX 3070 Ti / 8 GB) and the execution PC (RTX A4000+ / 16 GB+) run the **same code paths**. Differences are isolated to YAML overrides keyed by node name:

| Profile | Auto-selected when | Effect |
|---------|--------------------|--------|
| `dev_8gb` | total VRAM < 12 GiB | SAM2 small checkpoint, MegaPose `bsz_images=32`, BundleSDF `shorter_side=320`, FoundationPose 3 refine iters |
| `prod_16gb` | total VRAM ≥ 12 GiB | No overrides — every node uses its packaged YAML defaults |
| `cpu_only` | `nvidia-smi` not on PATH | All Phase 4 nodes flip to `mock` backend; YOLO `device:cpu` |

Profiles never branch code (Invariant I9). The runtime helper `resolve_torch_device()` is the second line of defence — it falls back to CPU even if a misconfigured profile points the node at an unusable GPU. Authoring guide: [`host_profiles/README.md`](../packages/infrastructure/perception_launch_utils/host_profiles/README.md).

## Topic reference

### Per-camera (namespaced)

| Topic | Type | Producer |
|-------|------|----------|
| `/{ns}/yolo/detections` | `DetectionArray` | `yolo_pcl_cpp_tracker` |
| `/{ns}/yolo_tracker/raw_poses` | `PoseWithMetaArray` | `yolo_pcl_cpp_tracker` |

`{ns}` is `cam0`, `cam1`, `cam2` in multi-camera launches.

### Cross-camera fusion

| Topic | Type | Producer |
|-------|------|----------|
| `/associated/poses` | `AssociatedPoseArray` | `cross_camera_associator` |
| `/merged/points` | `PointCloud2` | `pcl_merge_node` |

### Downstream (global frame)

| Topic | Type | Producer |
|-------|------|----------|
| `/pose_filter/filtered_poses` | `PoseWithMetaArray` | `pose_filter_cpp` |
| `/smoother/smoothed_poses` | `PoseWithMetaArray` | `pose_graph_smoother` |
| `/foundationpose/raw_poses` | `PoseWithMetaArray` | `isaac_foundationpose_tracker` |
| `/cosypose/optimized_poses` | `PoseWithMetaArray` | `cosypose_scene_optimizer` |
| `/megapose/raw_poses` | `PoseWithMetaArray` | `megapose_ros2_wrapper` |
| `/bundlesdf/raw_poses` | `PoseWithMetaArray` | `bundlesdf_unknown_tracker` |
| `/sam2/masks` | `SegmentationArray` | `sam2_instance_segmentor` |
| `/meta_controller/active_pipeline` | `PipelineStatus` | `perception_meta_controller` |

## TF2 frame convention

### Single camera

- Parent: `camera_color_optical_frame`
- Child:  `object_{track_id}_filtered`

### Multi-camera

- Per-camera: `cam{N}_color_optical_frame` (N ∈ {0, 1, 2})
- All filtered poses published in `ur5e_base_link`
- Static TF: `ur5e_base_link` → `cam{N}_link` (produced by `multi_camera_calibration`)
- Dynamic: `cam{N}_link` → `cam{N}_color_optical_frame` (from the camera driver)

## Pipeline modes

Switched via `perception_meta_controller` (`SetMode` service). See [running.md](running.md#pipeline-modes).

| Mode | Active nodes | Use case |
|------|--------------|----------|
| `NORMAL` | YOLO tracker + ICP + pose filter | Real-time tracking |
| `HIGH_PRECISION` | `NORMAL` + FoundationPose + pose graph smoother | High-accuracy manipulation |
| `SCENE_ANALYSIS` | YOLO tracker + SAM2 + CosyPose + pose filter | Scene understanding |

## QoS

- **Control-path topics** (poses, TF, internal `yolo/detections` / `sam2/masks` / `pose_filter/*` / `smoother/*` publishers): `BEST_EFFORT` + depth 1 (Invariant I7). A stale sample is worthless; dropping is preferable to queueing.
- **Camera-image input subscription**: matches the camera driver. `ros-jazzy-realsense2-camera 4.57.7` publishes `RELIABLE`, so the YOLO tracker defaults `image_qos:=reliable`. Pass `image_qos:=sensor_data` to the bringup / phase 1 / tracker launches when the driver uses `SensorDataQoS` (BEST_EFFORT). This is a publisher↔subscriber boundary, separate from the internal control-path policy.
- **Diagnostic / debug topics**: default reliable QoS.

## Optional dependencies

These have fallbacks. Packages must build and run sensibly without them.

| Library | Package | Fallback |
|---------|---------|----------|
| TEASER++ | `teaser_icp_hybrid_registrator` | ICP-only |
| Ceres | `multi_camera_calibration` | OpenCV-only solver |
| GTSAM | `pose_graph_smoother` | Passthrough |

## Cross-workspace boundary

An external robot-controller workspace integrates with this repo as a downstream consumer. The two workspaces share:

- **TF2 frames**: `ur5e_base_link` and related chain
- **Action interfaces**: `PlanGrasp`, `AnalyzeScene`

They do **not** share:

- C++ / Python imports
- Package.xml dependencies
- Custom messages beyond what is in `perception_msgs`
