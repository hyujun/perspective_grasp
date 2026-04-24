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
    )


def generate_launch_description():
    return LaunchDescription([
        declare_params_file_arg('sam2_instance_segmentor', 'sam2_params.yaml'),
        declare_camera_config_arg(),
        declare_autostart_arg(),
        OpaqueFunction(function=_expand),
    ])
```

`topic_overrides` is a callable `(namespace_clean) -> dict` — the only
thing that actually differs between the five Phase 4 launch files.

### `autostart_lifecycle_actions(node, autostart)`

Low-level helper used by `fanout_lifecycle_nodes`. Exposed for launch
files that spawn a single `LifecycleNode` outside of the fan-out path
(e.g., `smoother.launch.py`).

## Tests

```bash
colcon test --packages-select perception_launch_utils
colcon test-result --verbose
```

Pure-pytest, no rclcpp spin-up — covers YAML parsing edge cases, path
helpers, and the fan-out action-count invariant.
