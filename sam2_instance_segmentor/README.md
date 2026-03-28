# sam2_instance_segmentor

ROS 2 LifecycleNode wrapper for SAM2 (Segment Anything Model 2) instance segmentation.

## Overview

Generates precise instance segmentation masks for detected objects using SAM2. Takes YOLO bounding box detections as prompts and produces high-quality per-instance masks for downstream pose estimation and scene understanding.

**Status**: Stub implementation — lifecycle infrastructure is in place, SAM2 inference integration is pending.

## Node

**Node Name**: `sam2_segmentor` (LifecycleNode)

| | |
|---|---|
| **Subscribes** | `/camera/color/image_raw` (`Image`), `/yolo/detections` (`DetectionArray`) |
| **Publishes** | `/sam2/masks` (`SegmentationArray`) |

## Parameters

| Parameter | Default | Description |
|-----------|---------|-------------|
| `model_checkpoint` | `""` | Path to SAM2 model checkpoint |
| `model_config` | `sam2_hiera_l` | SAM2 model architecture variant |
| `points_per_side` | `32` | Prompt points per side |
| `pred_iou_thresh` | `0.88` | Prediction IoU threshold |
| `stability_score_thresh` | `0.95` | Mask stability threshold |
| `image_topic` | `/camera/color/image_raw` | Input image topic |

## Algorithm (Planned)

SAM2 (Hiera-Large variant) uses YOLO detection bounding boxes as prompts to generate refined instance segmentation masks, filtered by IoU and stability thresholds.

## Dependencies

- `rclpy`, `sensor_msgs`, `perception_msgs`

## Build

```bash
colcon build --packages-select sam2_instance_segmentor
```
