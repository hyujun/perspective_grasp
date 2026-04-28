"""Launch BundleSDF unknown-object tracker node(s).

Two modes, dispatched by ``camera_config``:

* **Single camera (default)** — one ``bundlesdf_tracker`` LifecycleNode
  in the global namespace with topics from ``config/bundlesdf_params.yaml``.
* **Multi-camera (fan-out)** — one LifecycleNode per
  ``perception_system.cameras`` entry, each namespaced under
  ``cam{id}`` with topics prefixed.

Each instance keeps its own per-track neural SDFs — they are not shared
across cameras.

``autostart`` (default ``true``) drives each LifecycleNode
``UNCONFIGURED → INACTIVE → ACTIVE`` so the Docker entrypoint yields an
active tracker without manual lifecycle commands.
"""

from __future__ import annotations

from launch import LaunchDescription
from launch.actions import OpaqueFunction
from launch.substitutions import LaunchConfiguration

from perception_launch_utils import (
    declare_autostart_arg,
    declare_camera_config_arg,
    declare_host_profile_arg,
    declare_params_file_arg,
    fanout_lifecycle_nodes,
    resolve_host_profile,
)


def _expand(context, *_args, **_kwargs):
    profile = resolve_host_profile(
        LaunchConfiguration('host_profile').perform(context))
    return fanout_lifecycle_nodes(
        package='bundlesdf_unknown_tracker',
        executable='bundlesdf_node',
        name='bundlesdf_tracker',
        params_file=LaunchConfiguration('params_file').perform(context),
        camera_config_path=(
            LaunchConfiguration('camera_config').perform(context)
        ),
        topic_overrides=lambda ns: {
            'image_topic':        f'/{ns}/camera/color/image_raw',
            'depth_topic':        f'/{ns}/camera/depth/image_rect_raw',
            'camera_info_topic':  f'/{ns}/camera/color/camera_info',
            'masks_topic':        f'/{ns}/sam2/masks',
            'poses_topic':        f'/{ns}/bundlesdf/raw_poses',
        },
        autostart=LaunchConfiguration('autostart'),
        host_profile=profile,
    )


def generate_launch_description():
    return LaunchDescription([
        declare_params_file_arg(
            'bundlesdf_unknown_tracker', 'bundlesdf_params.yaml'),
        declare_camera_config_arg(),
        declare_host_profile_arg(),
        declare_autostart_arg(),
        OpaqueFunction(function=_expand),
    ])
