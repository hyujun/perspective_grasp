# pcl_merge_node

Multi-camera point cloud merging, filtering, and deduplication.

## Overview

Merges depth point clouds from multiple eye-to-hand cameras into a single unified point cloud in the robot base frame. Performs spatial filtering, statistical outlier removal, voxel grid deduplication, and overlap scoring.

## Node

**Node Name**: `pcl_merge_node`

| | |
|---|---|
| **Subscribes** | Configurable via `source_topics` (`sensor_msgs/PointCloud2`) |
| **Publishes** | `/merged/points` (`PointCloud2`), `/merged/diagnostics` (`DiagnosticArray`) |

## Operation Modes

| Mode | Condition | Behavior |
|------|-----------|----------|
| **PASSTHROUGH** | 1 source topic | Preprocess only, no merging |
| **MERGE_2** | 2 source topics | ApproximateTime synchronized merging |
| **MERGE_N** | 3+ source topics | Timer-based merging at 10 Hz |

## Parameters

| Parameter | Default | Description |
|-----------|---------|-------------|
| `source_topics` | `["/cam1/camera/depth/points"]` | Input depth cloud topics |
| `target_frame` | `ur5e_base_link` | Output reference frame |
| `voxel_leaf_size` | `0.002` | Deduplication voxel size (m) |
| `table_height` | `0.0` | Table surface Z-coordinate (m) |
| `max_object_height` | `0.30` | Max object height above table (m) |
| `outlier_mean_k` | `30` | K neighbors for outlier removal |
| `outlier_stddev` | `1.5` | Stddev multiplier for outlier detection |
| `overlap_radius` | `0.005` | Radius for overlap scoring (m) |

## Processing Pipeline

1. **TF2 Transform** — Each cloud transformed to `target_frame`
2. **PassThrough Filter** — Removes points outside work region (Z-axis)
3. **Statistical Outlier Removal** — Filters noise based on local density
4. **Concatenation** — Merges all preprocessed clouds
5. **VoxelGrid Deduplication** — Reduces duplicate observations
6. **Overlap Scoring** — KD-tree radius search counts per-point camera observations

## Dependencies

- `rclcpp`, `sensor_msgs`, `tf2_ros`, `tf2_sensor_msgs`, `pcl_conversions`, `message_filters`, `diagnostic_msgs`
- PCL (common, filters, kdtree)

## Build

```bash
colcon build --packages-select pcl_merge_node
```
