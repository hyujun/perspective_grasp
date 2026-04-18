# teaser_icp_hybrid_registrator

Hybrid point cloud registration library combining TEASER++ global registration with ICP local refinement.

## Overview

A C++ library (no ROS 2 nodes) providing robust 6D pose estimation by aligning CAD model point clouds to sensor-captured scene point clouds. Supports two workflows:

- **Full pipeline** (`align()`) — TEASER++ global registration + ICP refinement for initial detection or recovery
- **ICP-only** (`refineIcp()`) — Local refinement with a known initial guess for frame-to-frame tracking

## API

```cpp
#include <teaser_icp_hybrid/hybrid_registrator.hpp>

perspective_grasp::HybridRegistrator registrator(config);

// Full pipeline (no prior pose needed)
auto result = registrator.align(source_cloud, target_cloud);

// ICP refinement only (tracking mode)
auto result = registrator.refineIcp(source_cloud, target_cloud, initial_guess);
```

### RegistrationResult

| Field | Type | Description |
|-------|------|-------------|
| `transform` | `Eigen::Isometry3d` | Output SE(3) rigid transformation |
| `teaser_fitness` | `double` | TEASER++ matching quality score |
| `icp_fitness` | `double` | ICP alignment error metric |
| `success` | `bool` | Convergence flag |
| `elapsed_ms` | `double` | Execution time |

### RegistrationConfig

| Parameter | Default | Description |
|-----------|---------|-------------|
| `voxel_size` | 0.005 | Voxel grid downsampling size (m) |
| `icp_max_iterations` | 50 | Maximum ICP iterations |
| `icp_transformation_epsilon` | 1e-8 | ICP convergence threshold |
| `icp_euclidean_fitness_epsilon` | 1e-6 | ICP fitness threshold |
| `icp_max_correspondence_distance` | 0.02 | Max correspondence distance (m) |
| `fpfh_normal_radius` | — | FPFH normal estimation radius |
| `fpfh_feature_radius` | — | FPFH feature computation radius |
| `noise_bound` | — | TEASER++ noise bound |

## Algorithms

1. **Voxel Grid Downsampling** — Reduces point density for computational efficiency
2. **FPFH Feature Extraction** — 33-dimensional descriptors for correspondence matching (multi-threaded OMP)
3. **TEASER++ Global Registration** — GNC-TLS robust rotation estimation, handles outlier correspondences
4. **ICP Local Refinement** — Point-to-point iterative closest point from PCL

## Dependencies

- **Required**: Eigen3, PCL (common, features, filters, registration, search, kdtree)
- **Optional**: TEASER++ (graceful fallback to ICP-only if unavailable, controlled by `HAS_TEASERPP` flag)

## Build

```bash
colcon build --packages-select teaser_icp_hybrid_registrator
```
