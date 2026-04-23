"""Launch BundleSDF unknown-object tracker node(s).

Two modes (same contract as ``foundationpose.launch.py`` /
``sam2_segmentor.launch.py``):

* **Single camera (default)** — no ``camera_config`` arg. Spawns one
  ``bundlesdf_tracker`` LifecycleNode in the global namespace with
  the topic defaults from ``config/bundlesdf_params.yaml``.
* **Multi-camera (fan-out)** — ``camera_config:=/path/to/camera_config.yaml``.
  Reads ``perception_system.cameras`` from the YAML and spawns one
  LifecycleNode per camera, each namespaced under ``cam{id}`` with its
  topics prefixed accordingly.

Each fan-out instance keeps its own per-track state; BundleSDF's
neural SDFs are not shared across cameras.

``autostart`` (default ``true``) drives each LifecycleNode from
``UNCONFIGURED`` → ``INACTIVE`` → ``ACTIVE`` automatically so the
Docker entrypoint yields an active tracker without manual lifecycle
commands.
"""

from __future__ import annotations

import os
from typing import Any

import lifecycle_msgs.msg
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchContext, LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    EmitEvent,
    OpaqueFunction,
    RegisterEventHandler,
)
from launch.conditions import IfCondition
from launch.events import matches_action
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import LifecycleNode
from launch_ros.event_handlers import OnStateTransition
from launch_ros.events.lifecycle import ChangeState


def _default_params_file() -> str:
    return os.path.join(
        get_package_share_directory('bundlesdf_unknown_tracker'),
        'config', 'bundlesdf_params.yaml')


def _single_node(params_file: str) -> LifecycleNode:
    return LifecycleNode(
        package='bundlesdf_unknown_tracker',
        executable='bundlesdf_node',
        name='bundlesdf_tracker',
        namespace='',
        parameters=[params_file],
        output='screen',
    )


def _per_camera_node(
    params_file: str, cam_id: int, namespace: str,
) -> LifecycleNode:
    ns = namespace.strip('/') if namespace else f'cam{cam_id}'
    prefix = f'/{ns}'
    overrides: dict[str, Any] = {
        'image_topic':        f'{prefix}/camera/color/image_raw',
        'depth_topic':        f'{prefix}/camera/depth/image_rect_raw',
        'camera_info_topic':  f'{prefix}/camera/color/camera_info',
        'masks_topic':        f'{prefix}/sam2/masks',
        'poses_topic':        f'{prefix}/bundlesdf/raw_poses',
    }
    return LifecycleNode(
        package='bundlesdf_unknown_tracker',
        executable='bundlesdf_node',
        name='bundlesdf_tracker',
        namespace=ns,
        parameters=[params_file, overrides],
        output='screen',
    )


def _autostart_actions(node: LifecycleNode, autostart: LaunchConfiguration):
    """Drive `node` UNCONFIGURED → INACTIVE → ACTIVE when autostart is true."""
    cond = IfCondition(autostart)
    configure = EmitEvent(
        event=ChangeState(
            lifecycle_node_matcher=matches_action(node),
            transition_id=lifecycle_msgs.msg.Transition.TRANSITION_CONFIGURE,
        ),
        condition=cond,
    )
    activate_on_inactive = RegisterEventHandler(
        OnStateTransition(
            target_lifecycle_node=node,
            start_state='configuring',
            goal_state='inactive',
            entities=[
                EmitEvent(event=ChangeState(
                    lifecycle_node_matcher=matches_action(node),
                    transition_id=(
                        lifecycle_msgs.msg.Transition.TRANSITION_ACTIVATE
                    ),
                )),
            ],
        ),
        condition=cond,
    )
    return [activate_on_inactive, configure]


def _expand(context: LaunchContext, *_args, **_kwargs):
    params_file = LaunchConfiguration('params_file').perform(context)
    camera_config = LaunchConfiguration('camera_config').perform(context)
    autostart = LaunchConfiguration('autostart')

    if not camera_config:
        nodes = [_single_node(params_file)]
    else:
        if not os.path.isfile(camera_config):
            raise FileNotFoundError(f'camera_config not found: {camera_config}')

        with open(camera_config, 'r') as f:
            doc = yaml.safe_load(f) or {}
        cameras = (
            doc.get('perception_system', {})
               .get('ros__parameters', {})
               .get('cameras', [])
        )
        if not cameras:
            raise ValueError(
                f'{camera_config} has no perception_system.ros__parameters.cameras'
            )

        nodes = []
        for cam in cameras:
            cam_id = int(cam.get('id'))
            namespace = cam.get('namespace', '') or ''
            if not namespace:
                nodes.append(_single_node(params_file))
            else:
                nodes.append(_per_camera_node(params_file, cam_id, namespace))

    actions: list = []
    for node in nodes:
        actions.append(node)
        actions.extend(_autostart_actions(node, autostart))
    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'params_file',
            default_value=_default_params_file(),
            description='Path to bundlesdf_params.yaml'),
        DeclareLaunchArgument(
            'camera_config',
            default_value='',
            description=(
                'Path to camera_config.yaml. When empty, spawns a single '
                'node with default (non-namespaced) topics. Otherwise '
                'spawns one node per entry in perception_system.cameras.'
            )),
        DeclareLaunchArgument(
            'autostart',
            default_value='true',
            description=(
                'Auto-drive the LifecycleNode(s) to ACTIVE on launch. '
                'Set to false to manage the lifecycle externally.'
            )),
        OpaqueFunction(function=_expand),
    ])
