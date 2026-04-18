# perception_msgs

Custom ROS 2 message, service, and action definitions for the perspective_grasp 6D pose estimation and manipulation pipeline.

## Overview

This package defines all shared data structures used across the perception pipeline, including 2D detections, 6D poses with metadata, instance segmentation masks, cross-camera associations, filtered poses with covariance, and pipeline control interfaces.

## Message Types

| Message | Description |
|---------|-------------|
| `Detection` | Single 2D detection (YOLO + ByteTrack): object ID, class name, bounding box, confidence |
| `DetectionArray` | Array of `Detection` messages with header |
| `PoseWithMeta` | 6D pose with source metadata (yolo_tracker / foundationpose / megapose / bundlesdf) |
| `PoseWithMetaArray` | Array of `PoseWithMeta` messages with header |
| `PoseCovarianceStamped` | Filtered pose with 6x6 SE(3) covariance matrix |
| `PoseCovarianceArray` | Array of `PoseCovarianceStamped` messages |
| `Segmentation` | Instance segmentation mask (mono8 binary image + bounding box + confidence) |
| `SegmentationArray` | Array of `Segmentation` messages |
| `AssociatedPose` | Cross-camera associated pose with global ID, observing camera metadata |
| `AssociatedPoseArray` | Array of `AssociatedPose` messages |
| `ObjectRelation` | Spatial relationship between two objects (support / contact / adjacent) |
| `PipelineStatus` | Pipeline mode, active nodes, GPU usage, tracked/lost object counts |

## Service Types

| Service | Description |
|---------|-------------|
| `SetMode` | Switch perception pipeline mode (NORMAL / HIGH_PRECISION / SCENE_ANALYSIS) |

## Action Types

| Action | Description |
|--------|-------------|
| `AnalyzeScene` | Scene-level pose optimization with ROI and class filtering. Feedback: progress + stage |
| `PlanGrasp` | Grasp planning for target object. Returns grasp pose, 10-DoF finger config, approach waypoints |

## Dependencies

- `std_msgs`, `geometry_msgs`, `sensor_msgs`, `action_msgs`

## Build

```bash
colcon build --packages-select perception_msgs
```

> **Note**: This package must be built first as all other packages depend on it.
