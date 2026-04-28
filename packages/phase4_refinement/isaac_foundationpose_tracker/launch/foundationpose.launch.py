"""Launch FoundationPose tracker node(s).

Two modes, dispatched by ``camera_config``:

* **Single camera (default)** — one ``foundationpose_tracker``
  LifecycleNode in the global namespace with topic defaults from
  ``config/foundationpose_params.yaml``.
* **Multi-camera (fan-out)** — one LifecycleNode per
  ``perception_system.cameras`` entry, each namespaced under ``cam{id}``
  with topics prefixed.

Each instance loads its own mesh registry on ``on_activate``.

``autostart`` (default ``true``) drives each LifecycleNode
``UNCONFIGURED → INACTIVE → ACTIVE`` so the Docker entrypoint yields an
active node rather than one stuck in ``UNCONFIGURED``.
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
        package='isaac_foundationpose_tracker',
        executable='foundationpose_node',
        name='foundationpose_tracker',
        params_file=LaunchConfiguration('params_file').perform(context),
        camera_config_path=(
            LaunchConfiguration('camera_config').perform(context)
        ),
        topic_overrides=lambda ns: {
            'image_topic':        f'/{ns}/camera/color/image_raw',
            'depth_topic':        f'/{ns}/camera/depth/image_rect_raw',
            'camera_info_topic':  f'/{ns}/camera/color/camera_info',
            'detections_topic':   f'/{ns}/yolo/detections',
            'poses_topic':        f'/{ns}/foundationpose/raw_poses',
        },
        autostart=LaunchConfiguration('autostart'),
        host_profile=profile,
    )


def generate_launch_description():
    return LaunchDescription([
        declare_params_file_arg(
            'isaac_foundationpose_tracker', 'foundationpose_params.yaml'),
        declare_camera_config_arg(),
        declare_host_profile_arg(),
        declare_autostart_arg(),
        OpaqueFunction(function=_expand),
    ])
