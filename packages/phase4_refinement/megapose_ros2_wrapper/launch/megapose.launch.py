"""Launch MegaPose tracker node(s).

Two modes (same contract as ``sam2_segmentor.launch.py`` /
``foundationpose.launch.py``):

* **Single camera (default)** — no ``camera_config`` arg. Spawns one
  ``megapose_tracker`` LifecycleNode in the global namespace with
  topic defaults from ``config/megapose_params.yaml``.
* **Multi-camera (fan-out)** — ``camera_config:=/path/to/camera_config.yaml``.
  Reads ``perception_system.cameras`` from the YAML and spawns one
  LifecycleNode per camera, each namespaced under ``cam{id}``.

Each fan-out instance gets its own backend (its own mesh registry,
its own GPU allocation); meshes are loaded once per node on
``on_activate``.
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
        get_package_share_directory('megapose_ros2_wrapper'),
        'config', 'megapose_params.yaml')


def _single_node(params_file: str) -> LifecycleNode:
    return LifecycleNode(
        package='megapose_ros2_wrapper',
        executable='megapose_node',
        name='megapose_tracker',
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
        'camera_info_topic':  f'{prefix}/camera/color/camera_info',
        'detections_topic':   f'{prefix}/yolo/detections',
        'poses_topic':        f'{prefix}/megapose/raw_poses',
    }
    return LifecycleNode(
        package='megapose_ros2_wrapper',
        executable='megapose_node',
        name='megapose_tracker',
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
            description='Path to megapose_params.yaml'),
        DeclareLaunchArgument(
            'camera_config',
            default_value='',
            description=(
                'Path to camera_config.yaml. When empty, spawns a single '
                'node with default (non-namespaced) topics. Otherwise '
                'spawns one node per entry in perception_system.cameras.'
            )),
        OpaqueFunction(function=_expand),
    ])
