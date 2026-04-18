# Build

Covers compiling the colcon workspace and rebuilding Docker images.

## Full workspace build

[build.sh](../build.sh) wraps `colcon build` and phases the compile in 4 steps so inter-package dependencies resolve cleanly. It uses `$(nproc) - 2` parallel workers.

```bash
cd ~/ros2_ws/perspective_ws

# Default: RelWithDebInfo
./src/perspective_grasp/build.sh

# Release
./src/perspective_grasp/build.sh Release

# Clean first (wipes build/install/log)
./src/perspective_grasp/build.sh --clean
./src/perspective_grasp/build.sh --clean Release

source install/setup.bash
```

## Build order

`build.sh` runs these phases in order; each phase re-sources `install/setup.bash` before the next:

| # | Packages | Why |
|---|----------|-----|
| 1 | `perception_msgs` | All packages depend on custom messages/actions |
| 2 | `teaser_icp_hybrid_registrator` | C++ library consumed by `yolo_pcl_cpp_tracker` |
| 3 | `cross_camera_associator`, `pcl_merge_node` | Multi-camera fusion infrastructure |
| 4 | Everything else | Parallel-safe |

## Single-package build

When iterating on one package, skip `build.sh`:

```bash
cd ~/ros2_ws/perspective_ws
colcon build --packages-select <package_name>
source install/setup.bash
```

Examples:

```bash
colcon build --packages-select yolo_pcl_cpp_tracker
colcon build --packages-select pose_filter_cpp
colcon build --packages-select grasp_pose_planner
```

> Only fall back to `build.sh` when you touch `perception_msgs` or `teaser_icp_hybrid_registrator` (everything downstream needs rebuilding), or when you add a new package.

## Rebuilding Docker images

The ML containers compile `perception_msgs` at image-build time (via a `msgs-builder` stage inside [docker/Dockerfile](../docker/Dockerfile)). **Any change to `perception_msgs` requires rebuilding the image.**

```bash
cd ~/ros2_ws/perspective_ws/src/perspective_grasp

# Rebuild base image (foundationpose / cosypose / bundlesdf)
docker compose -f docker/docker-compose.yml build

# Rebuild a specific service
docker compose -f docker/docker-compose.yml build foundationpose
docker compose -f docker/docker-compose.yml build sam2        # uses sam2-runtime target

# Force no-cache (after a messy dependency change)
docker compose -f docker/docker-compose.yml build --no-cache
```

## Code conventions

- **C++ Standard**: C++20 (`cxx_std_20`)
- **Compiler flags**: `-Wall -Wextra -Wpedantic -Wshadow -Wconversion`
- **C++ build system**: `ament_cmake`
- **Python build system**: `ament_cmake_python` (mixed) or `ament_python`
- **Formatting**: clang-format (Google style with modifications)
- **Namespace**: `perspective_grasp`

## Troubleshooting

| Symptom | Fix |
|---------|-----|
| `perception_msgs/msg/...` not found | Build phase 1 first, then re-source `install/setup.bash` |
| TEASER++ link error | Re-run `scripts/install_dependencies.sh`; check `ldconfig -p \| grep teaserpp` |
| GTSAM not found at link time | Package has optional GTSAM — confirm it was intended; re-run dep script |
| `nvidia-container-cli` error on `docker compose build` | `sudo systemctl restart docker`, confirm `nvidia-ctk runtime configure --runtime=docker` ran |
| Stale Docker build after editing a Python node | Rebuild only if deps changed — source code is bind-mounted at runtime |
| Stale Docker build after editing `perception_msgs` | Rebuild with `docker compose build` (msgs are baked into the image) |

## Next

- [running.md](running.md) — launch the pipeline
- [architecture.md](architecture.md) — pipeline / topic / TF reference
