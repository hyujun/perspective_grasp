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

## Library targets

Two libraries plus a thin executable so the node class can be linked into the smoke test without
pulling in the executable's `main()`:

| Target | Source | Purpose |
|---|---|---|
| `pcl_merge_lib` | `cloud_preprocessor.cpp`, `overlap_filter.cpp` | Pure-logic PCL pipeline stages (PassThrough / SOR / KD-tree overlap scoring) |
| `pcl_merge_node_core` | `merge_node.cpp` | `MergeNode` class (rclcpp, message_filters, TF2); links `pcl_merge_lib` |
| `merge_node` (exe) | `merge_main.cpp` | Thin `rclcpp::init` + `spin` wrapper |

## Build

```bash
colcon build --packages-select pcl_merge_node
```

## Tests

`ament_cmake_gtest` — 23 cases across 3 binaries:

| Binary | Cases | What it covers |
|---|---:|---|
| `test_cloud_merger` | 8 | Empty / null input, all-outside-Z-range → empty, mixed in/out range filtering, SOR drops noise outliers, `setParams()` reflects on next `process`, `table_height` shift re-windows Z, VoxelGrid dedup ratio on merged duplicates |
| `test_overlap_filter` | 8 | Non-overlapping → all count=1, identical → all count=2, partial overlap mixes counts, empty merged, null merged, one source null → counts in {0,1}, shrinking radius reduces count, `getOverlapCount` out-of-range returns 0 |
| `test_merge_node_smoke` | 4 | `source_topics` size 1 / 2 / 3 selects PASSTHROUGH / MERGE_2 / MERGE_N mode, `/merged/points` advertised; drives a `SingleThreadedExecutor` for ~200 ms per case |

```bash
colcon test --packages-select pcl_merge_node
colcon test-result --verbose

# Drop into a single binary for gdb / --gtest_filter
./build/pcl_merge_node/test_overlap_filter --gtest_filter=OverlapFilter.Shrinking*
```

Tests run against synthetic PCL clouds (procedural grid + dense noise cluster helpers) — no
camera, GPU, or live ROS graph required.
