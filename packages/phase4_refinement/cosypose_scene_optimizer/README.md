# cosypose_scene_optimizer

ROS 2 action server for scene-level 6D pose optimization using CosyPose.

## Overview

Performs joint multi-object pose optimization by reasoning about scene-level spatial constraints and object-object relations. Operates as an on-demand action server (heavy computation, non-blocking).

**Status**: Stub implementation — action server skeleton is in place, CosyPose algorithm integration is pending.

## Node

**Node Name**: `cosypose_optimizer`

| | |
|---|---|
| **Action Server** | `analyze_scene` (`AnalyzeScene`) |
| **Publishes** | `/cosypose/optimized_poses` (`PoseWithMetaArray`) |

### AnalyzeScene Action

- **Goal**: ROI polygon points, target class filter
- **Result**: Optimized poses, object relations, success flag
- **Feedback**: Progress (0.0-1.0), current stage (coarse / refine / optimize)

## Parameters

| Parameter | Default | Description |
|-----------|---------|-------------|
| `model_dir` | `""` | Path to CosyPose model weights |
| `n_coarse_iterations` | `1` | Coarse optimization iterations |
| `n_refine_iterations` | `4` | Refinement iterations |
| `n_pose_hypotheses` | `5` | Pose hypotheses per object |
| `bsz_objects` | `16` | Object batch size for GPU inference |

## Algorithm (Planned)

CosyPose jointly optimizes multiple object poses at the scene level, inferring spatial relations (support, contact, adjacency) and enforcing physical plausibility constraints.

## Dependencies

- `rclpy`, `geometry_msgs`, `perception_msgs`

## Build

```bash
colcon build --packages-select cosypose_scene_optimizer
```
