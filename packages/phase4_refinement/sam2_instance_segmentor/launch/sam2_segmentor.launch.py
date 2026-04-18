"""Launch SAM2 instance segmentor node(s).

Two modes:

* **Single camera (default)** — no ``camera_config`` arg. Spawns one
  ``sam2_segmentor`` LifecycleNode in the global namespace with the
  topic defaults from ``config/sam2_params.yaml``. Preserves existing
  docker-compose / ``test_live.py`` behavior.

* **Multi-camera (fan-out)** — ``camera_config:=/path/to/camera_config.yaml``.
  Reads ``perception_system.cameras`` from the YAML and spawns one
  LifecycleNode per camera, each namespaced under ``cam{id}`` with its
  topics prefixed accordingly (``/cam{id}/camera/color/image_raw`` →
  ``/cam{id}/yolo/detections`` → ``/cam{id}/sam2/masks``). Matches the
  CLAUDE.md multi-camera convention (``/{ns}/yolo/detections`` etc.).

The same container can host all instances — each instance loads its
own backend weights (SAM2 checkpoint), which is the intended isolation
on the production GPU.
"""

from __future__ import annotations

import os
from typing import Any

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchContext, LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import LifecycleNode


def _default_params_file() -> str:
    return os.path.join(
        get_package_share_directory('sam2_instance_segmentor'),
        'config', 'sam2_params.yaml')


def _single_node(params_file: str) -> LifecycleNode:
    return LifecycleNode(
        package='sam2_instance_segmentor',
        executable='sam2_segmentor_node',
        name='sam2_segmentor',
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
        'image_topic':      f'{prefix}/camera/color/image_raw',
        'detections_topic': f'{prefix}/yolo/detections',
        'masks_topic':      f'{prefix}/sam2/masks',
    }
    return LifecycleNode(
        package='sam2_instance_segmentor',
        executable='sam2_segmentor_node',
        name='sam2_segmentor',
        namespace=ns,
        parameters=[params_file, overrides],
        output='screen',
    )


def _expand(context: LaunchContext, *_args, **_kwargs):
    params_file = LaunchConfiguration('params_file').perform(context)
    camera_config = LaunchConfiguration('camera_config').perform(context)

    if not camera_config:
        return [_single_node(params_file)]

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

    nodes: list[LifecycleNode] = []
    for cam in cameras:
        cam_id = int(cam.get('id'))
        namespace = cam.get('namespace', '') or ''
        # Empty namespace (1-cam config) → keep the global-topic behavior.
        if not namespace:
            nodes.append(_single_node(params_file))
        else:
            nodes.append(_per_camera_node(params_file, cam_id, namespace))
    return nodes


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'params_file',
            default_value=_default_params_file(),
            description='Path to sam2_params.yaml'),
        DeclareLaunchArgument(
            'camera_config',
            default_value='',
            description=(
                'Path to camera_config.yaml. When empty, spawns a single '
                'node with the default (non-namespaced) topics. Otherwise '
                'spawns one node per entry in perception_system.cameras.'
            )),
        OpaqueFunction(function=_expand),
    ])
