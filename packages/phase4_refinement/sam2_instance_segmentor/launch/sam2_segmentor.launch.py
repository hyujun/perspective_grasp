"""Launch SAM2 instance segmentor node(s).

Two modes, dispatched by ``camera_config``:

* **Single camera (default)** — no ``camera_config`` arg. One
  ``sam2_segmentor`` LifecycleNode in the global namespace, topics
  from ``config/sam2_params.yaml``.
* **Multi-camera (fan-out)** — ``camera_config:=/path/to/camera_config.yaml``.
  One LifecycleNode per ``perception_system.cameras`` entry, each
  namespaced under ``cam{id}`` with topics prefixed accordingly
  (``/cam{id}/camera/color/image_raw`` → ``/cam{id}/yolo/detections`` →
  ``/cam{id}/sam2/masks``).

Each instance loads its own SAM2 checkpoint on ``on_activate``.

``autostart`` (default ``true``) drives each LifecycleNode
``UNCONFIGURED → INACTIVE → ACTIVE`` so the Docker entrypoint
(``ros2 launch ...``) yields an already-active node.
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
        host_profile=profile,
    )


def generate_launch_description():
    return LaunchDescription([
        declare_params_file_arg('sam2_instance_segmentor', 'sam2_params.yaml'),
        declare_camera_config_arg(),
        declare_host_profile_arg(),
        declare_autostart_arg(),
        OpaqueFunction(function=_expand),
    ])
