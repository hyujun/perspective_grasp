# perception_bringup

System-level launches and shared configuration for the `perspective_grasp`
pipeline. Imports from `perception_launch_utils` rather than hand-rolling
config-path / camera-config / lifecycle wiring (see CLAUDE.md §9).

## Launch files

| File | Spawns |
|------|--------|
| [`perception_system.launch.py`](launch/perception_system.launch.py) | Per-camera Phase 1 (yolo + pcl_icp), then shared Phase 2 (associator) + Phase 3 (filter, smoother). One-liner for the host-side stack. |
| [`phase1_bringup.launch.py`](launch/phase1_bringup.launch.py) | Phase 1 only (per-camera), no fusion / filtering. Useful for tuning ICP / detection in isolation. |

`perception_system.launch.py` accepts:

| Arg | Default | Effect |
|-----|---------|--------|
| `camera_config` | `''` | Path to a `camera_config*.yaml`. Empty → 1-cam fallback. Otherwise spawns one per-camera subtree per entry. |
| `host_profile` | `auto` | `auto` / `dev_8gb` / `prod_16gb` / `cpu_only`. Selects parameter overrides keyed by node name; YAMLs ship with `perception_launch_utils` ([`host_profiles/`](../../infrastructure/perception_launch_utils/host_profiles/)). Env override: `PERSPECTIVE_HOST_PROFILE`. |
| `preflight` | `true` | Print a `preflight:` block (driver / torch / CUDA versions) at launch start. Set `false` or `PERSPECTIVE_PREFLIGHT_SKIP=1` to bypass. |
| `preflight_strict` | `false` | When the probe reports a hard error, abort launch instead of warning. |

```bash
# Single camera, defaults
ros2 launch perception_bringup perception_system.launch.py

# Two cameras, force dev profile (e.g. 8 GB box)
ros2 launch perception_bringup perception_system.launch.py \
    camera_config:=$(ros2 pkg prefix perception_bringup)/share/perception_bringup/config/camera_config_2cam.yaml \
    host_profile:=dev_8gb

# CI-style headless run (no GPU)
PERSPECTIVE_HOST_PROFILE=cpu_only PERSPECTIVE_PREFLIGHT_SKIP=1 \
    ros2 launch perception_bringup perception_system.launch.py
```

## Configuration files

| File | Purpose |
|------|---------|
| [`config/camera_config.yaml`](config/camera_config.yaml) | 3-camera default (`/cam0` / `/cam1` / `/cam2`) |
| [`config/camera_config_1cam.yaml`](config/camera_config_1cam.yaml) | Single camera (root namespace, no fan-out) |
| [`config/camera_config_2cam.yaml`](config/camera_config_2cam.yaml) | 2-camera (`/cam0` + `/cam1`) |
| [`config/cyclonedds_localhost.xml`](config/cyclonedds_localhost.xml) | XML peer config sourced by `.env.live` to lockstep host & Phase 4 containers on Cyclone DDS |

Host-profile YAMLs (`dev_8gb` / `prod_16gb` / `cpu_only`) ship with
`perception_launch_utils` at
[`packages/infrastructure/perception_launch_utils/host_profiles/`](../../infrastructure/perception_launch_utils/host_profiles/),
not under this package — co-located with the loader so Phase 4 Docker
images don't need to install `perception_bringup` to resolve a profile.

Robot-agnostic frame names live in `camera_config*.yaml`
(`base`, `tool0`); the static TF from `ur5e_base_link` is applied
elsewhere — keeps vision config reusable across arms (CLAUDE.md
anti-pattern (l)).

## Multi-camera convention

The launch path is unified: `N=1` is "N=1 multi-camera", not a special
case. Per-camera nodes are namespaced `/cam0/`, `/cam1/`, … (Invariant
I4). Don't add `if N == 1` branches — change the YAML instead.

## Dependencies

- **ROS**: `rclcpp`, all Phase 1–3 packages, `perception_launch_utils`
- **Pure ament_cmake** — no Python build target, but the launch files
  themselves use Python and import from `perception_launch_utils`.

## Tests

This package has no unit tests of its own; it's exercised by the
integration smoke tests called out in CLAUDE.md §4.
