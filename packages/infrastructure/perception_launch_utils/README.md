# perception_launch_utils

Shared Python helpers for every `launch.py` in the `perspective_grasp`
workspace. Collapses three repeating patterns:

1. **Config-file path lookup** — the `os.path.join(get_package_share_directory(...), 'config', '<pkg>_params.yaml')` block that appeared in 8 launch files.
2. **`camera_config*.yaml` loading** — the two-callsite `importlib` dance used to reach `perception_bringup/launch/camera_config_loader.py` is replaced by a normal `from perception_launch_utils import load_config`.
3. **Lifecycle fan-out (Phase 4)** — the ~140 lines of identical `OpaqueFunction + yaml.safe_load + per-camera LifecycleNode + autostart event wiring` that appeared in `cosypose`, `bundlesdf`, `megapose`, `sam2`, `foundationpose` collapse to one `fanout_lifecycle_nodes(...)` call.

## Install

It's a plain `ament_python` package — add it to `package.xml`:

```xml
<exec_depend>perception_launch_utils</exec_depend>
```

## API

### `config_path(pkg, filename=None) -> str`

```python
from perception_launch_utils import config_path

config = config_path('pose_filter_cpp')
# → <ws>/install/pose_filter_cpp/share/pose_filter_cpp/config/pose_filter_cpp_params.yaml
config = config_path('pose_filter_cpp', 'filter_params.yaml')
```

### `share_file(pkg, *relative_parts) -> str`

```python
from perception_launch_utils import share_file

rviz = share_file('perception_debug_visualizer', 'rviz', 'debug_view.rviz')
```

### `repo_root(anchor_pkg='perception_bringup') -> str`

Absolute path to `<ws>/src/perspective_grasp`. Resolves via the anchor
package's installed `share/` dir (walks up 4 levels). Override with
`$PERSPECTIVE_GRASP_REPO_ROOT`. If the source folder was renamed (e.g. on
a deployment PC) and the env var is unset, the fallback path will not
exist and the call raises `FileNotFoundError` with a hint pointing at
the env var — set it from `.env.live` to recover.

### `workspace_models_dir(anchor_pkg='perception_bringup') -> str`

Absolute path to `<repo>/models` — the home for input assets (pre-positioned
weights, meshes, datasets, plus YOLO's auto-downloaded `yolov8n.pt`).
Override with `$PERSPECTIVE_GRASP_MODELS_DIR`.

```python
from perception_launch_utils import workspace_models_dir

parameters=[{
    'model_path': 'yolov8n.pt',
    'models_dir': workspace_models_dir('yolo_pcl_cpp_tracker'),
}],
```

### `workspace_runtime_outputs_dir(subdir='', *, anchor_pkg='perception_bringup', create=True) -> str`

Absolute path to `<repo>/runtime_outputs[/<subdir>]` — the home for
artifacts a node produces at runtime (calibration results, BundleSDF dumps,
debug recordings). Distinct from `models/`. Auto-creates the directory.
Override with `$PERSPECTIVE_GRASP_RUNTIME_OUTPUTS_DIR`. Both `models/` and
`runtime_outputs/` are git-ignored at the repo root.

```python
from perception_launch_utils import workspace_runtime_outputs_dir

# In a launch file: inject the workspace-rooted output dir as a parameter.
Node(
    package='multi_camera_calibration',
    executable='collect_calibration_data.py',
    parameters=[
        LaunchConfiguration('params_file'),
        {'output_dir': workspace_runtime_outputs_dir('calibration')},
    ],
)
```

### `declare_{params_file,camera_config,autostart}_arg()`

One-line replacements for the standard `DeclareLaunchArgument` stanzas.

```python
from perception_launch_utils import (
    declare_params_file_arg,
    declare_camera_config_arg,
    declare_autostart_arg,
)

return LaunchDescription([
    declare_params_file_arg('cosypose_scene_optimizer'),
    declare_camera_config_arg(),
    declare_autostart_arg(),
    ...
])
```

### `load_config(path) -> PerceptionSystemConfig`

Typed loader for `camera_config*.yaml`. Empty/missing path falls back to
a 1-cam single-namespace config (matches `camera_config_1cam.yaml`):

```python
from perception_launch_utils import load_config

cfg = load_config('/path/to/camera_config.yaml')
for cam in cfg.cameras:
    print(cam.id, cam.namespace, cam.frame_id)
print(cfg.base_frame)          # 'ur5e_base_link'
print(cfg.namespaces)           # ['/cam0', '/cam1', ...]
```

### `fanout_lifecycle_nodes(...)`

Single call replacing the Phase 4 boilerplate. Single node when
`camera_config_path` is empty; one node per camera entry otherwise.

```python
from launch import LaunchDescription
from launch.actions import OpaqueFunction
from launch.substitutions import LaunchConfiguration
from perception_launch_utils import (
    declare_params_file_arg,
    declare_camera_config_arg,
    declare_autostart_arg,
    fanout_lifecycle_nodes,
)


def _expand(context, *_a, **_kw):
    profile = resolve_host_profile(
        LaunchConfiguration('host_profile').perform(context))
    return fanout_lifecycle_nodes(
        package='sam2_instance_segmentor',
        executable='sam2_segmentor_node',
        name='sam2_segmentor',
        params_file=LaunchConfiguration('params_file').perform(context),
        camera_config_path=(
            LaunchConfiguration('camera_config').perform(context)
        ),
        topic_overrides=lambda ns: {
            'image_topic':      f'/{ns}/camera/color/image_raw',
            'detections_topic': f'/{ns}/yolo/detections',
            'masks_topic':      f'/{ns}/sam2/masks',
        },
        autostart=LaunchConfiguration('autostart'),
        host_profile=profile,        # ← swap model size / batch by host class
    )


def generate_launch_description():
    return LaunchDescription([
        declare_params_file_arg('sam2_instance_segmentor', 'sam2_params.yaml'),
        declare_camera_config_arg(),
        declare_host_profile_arg(),
        declare_autostart_arg(),
        OpaqueFunction(function=_expand),
    ])
```

`topic_overrides` is a callable `(namespace_clean) -> dict` — the only
thing that actually differs between the five Phase 4 launch files.

`host_profile` (optional) plumbs the resolved
[`HostProfile`](#host-profiles--resolve_host_profile--declare_host_profile_arg)
through; per-node overrides are looked up by the `name=` field. Callers
that don't need profiles can omit it entirely — behaviour is identical
to the pre-profile API.

### `autostart_lifecycle_actions(node, autostart)`

Low-level helper used by `fanout_lifecycle_nodes`. Exposed for launch
files that spawn a single `LifecycleNode` outside of the fan-out path
(e.g., `smoother.launch.py`).

### `resolve_torch_device(requested, logger) -> DeviceResolution`

Used by every GPU node in the workspace (yolo_byte_tracker + Phase 4
backends). Maps a YAML `device` parameter to a verified torch device
string, falling back to `"cpu"` with a single WARN when CUDA is
unusable. Catches the dev-PC ↔ execution-PC trap where the host venv
was built against a different CUDA than the driver supports (e.g.
`cu130` torch wheel on a CUDA-12.8 driver — `is_available()` lies,
allocation fails). The probe allocates a 1-element tensor on the
target device so the failure surfaces here instead of inside a model's
forward pass.

```python
from perception_launch_utils import resolve_torch_device

# In a node's load() — pass the live torch module so we don't import twice.
import torch
res = resolve_torch_device(
    self._requested_device, self._node.get_logger(), torch_mod=torch,
)
self._device = res.device              # 'cuda:0' or 'cpu' — pass to .to() / build_sam2()
device_type = res.device_type          # 'cuda' or 'cpu' — pass to torch.autocast(...)
if res.downgraded:
    ...                                 # optional: skip half-precision paths on CPU
```

Accepted `requested` values:

| Input            | Resolved (CUDA OK) | Resolved (no CUDA) |
|------------------|--------------------|--------------------|
| `"auto"` (default) | `cuda:0`         | `cpu` (warn)       |
| `"cuda"`         | `cuda:0`           | `cpu` (warn)       |
| `"cuda:N"` / `"N"` | `cuda:N`         | `cpu` (warn)       |
| `"cpu"`          | `cpu`              | `cpu`              |

### `check_host_environment()` / `preflight_launch_action(strict=...)`

Once-per-launch probe of `nvidia-smi` + `torch` versions. Logs one
formatted block at the top of the launch log; warns on driver/torch
CUDA mismatch (the cu130-on-12.8-driver case). `preflight_launch_action`
wraps the probe in an `OpaqueFunction`-compatible callable that returns
`[LogInfo(...)]` (and `Shutdown` when `strict=True` and an error was
detected). Set `PERSPECTIVE_PREFLIGHT_SKIP=1` to bypass without editing
the launch file. See `perception_system.launch.py` for the canonical
wiring (`preflight` / `preflight_strict` launch args).

### Host profiles — `resolve_host_profile()` / `declare_host_profile_arg()`

Per-host parameter overrides live in
`packages/bringup/perception_bringup/config/host_profiles/`. Three
profiles ship: `dev_8gb`, `prod_16gb`, `cpu_only`. `auto` selects via
`nvidia-smi` total VRAM (`< 12000 MiB → dev_8gb`,
`≥ 12000 MiB → prod_16gb`, no GPU → `cpu_only`).

```python
from perception_launch_utils import (
    declare_host_profile_arg,
    overrides_for_node,
    resolve_host_profile,
)
from launch.substitutions import LaunchConfiguration

def _spawn(context, *_a, **_kw):
    profile = resolve_host_profile(
        LaunchConfiguration('host_profile').perform(context))
    sam2_overrides = overrides_for_node(profile, 'sam2_segmentor')
    return [Node(
        package='sam2_instance_segmentor',
        executable='sam2_segmentor_node',
        name='sam2_segmentor',
        parameters=[base_yaml_path, sam2_overrides] if sam2_overrides
                   else [base_yaml_path],
    )]

return LaunchDescription([
    declare_host_profile_arg(),
    OpaqueFunction(function=_spawn),
])
```

`overrides_for_node` returns `{}` when the profile doesn't mention the
node — splatting an empty dict into `parameters=[...]` is a safe no-op,
so callers never special-case the no-override path.

Override priority: launch arg `host_profile:=...` → env
`PERSPECTIVE_HOST_PROFILE` → `auto`.

## Tests

```bash
colcon test --packages-select perception_launch_utils
colcon test-result --verbose
```

Pure-pytest, no rclcpp spin-up — covers YAML parsing edge cases, path
helpers, and the fan-out action-count invariant.
