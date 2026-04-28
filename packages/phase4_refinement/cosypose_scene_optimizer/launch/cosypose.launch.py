"""Launch CosyPose scene optimizer node(s).

Two modes, dispatched by ``camera_config``:

* **Single camera (default)** — one ``cosypose_optimizer`` LifecycleNode
  in the global namespace with topics / action from
  ``config/cosypose_params.yaml``.
* **Multi-camera (fan-out)** — one LifecycleNode per
  ``perception_system.cameras`` entry, each namespaced under
  ``cam{id}`` with topics + ``analyze_scene`` action prefixed.

Each instance loads happypose weights once on ``on_activate``.

``autostart`` (default ``true``) drives each LifecycleNode
``UNCONFIGURED → INACTIVE → ACTIVE`` so ``analyze_scene`` goals can be
dispatched immediately after ``ros2 launch`` bootstrap finishes.
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
        package='cosypose_scene_optimizer',
        executable='cosypose_node',
        name='cosypose_optimizer',
        params_file=LaunchConfiguration('params_file').perform(context),
        camera_config_path=(
            LaunchConfiguration('camera_config').perform(context)
        ),
        topic_overrides=lambda ns: {
            'image_topic':        f'/{ns}/camera/color/image_raw',
            'camera_info_topic':  f'/{ns}/camera/color/camera_info',
            'detections_topic':   f'/{ns}/yolo/detections',
            'poses_topic':        f'/{ns}/cosypose/optimized_poses',
            'action_name':        f'/{ns}/analyze_scene',
        },
        autostart=LaunchConfiguration('autostart'),
        host_profile=profile,
    )


def generate_launch_description():
    return LaunchDescription([
        declare_params_file_arg(
            'cosypose_scene_optimizer', 'cosypose_params.yaml'),
        declare_camera_config_arg(),
        declare_host_profile_arg(),
        declare_autostart_arg(),
        OpaqueFunction(function=_expand),
    ])
