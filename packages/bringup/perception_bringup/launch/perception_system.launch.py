"""Config-driven multi-camera perception system launch.

Reads ``camera_config*.yaml`` and auto-generates per-camera tracking nodes
plus shared fusion / smoothing / debug nodes.

Usage::

  ros2 launch perception_bringup perception_system.launch.py \\
      camera_config:=/path/to/camera_config.yaml

When ``camera_config`` is empty, falls back to a single root-namespace camera
(identical to ``camera_config_1cam.yaml``).
"""

from launch import LaunchDescription
from launch.actions import GroupAction, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, PushRosNamespace

from perception_launch_utils import (
    config_path,
    declare_camera_config_arg,
    load_config,
)


def _per_camera_nodes(ns: str, tracker_config: str):
    """Build the per-camera subtree (YOLO + PCL-ICP pose estimator).

    Empty ``ns`` (1-cam) → nodes live in the root namespace.
    Non-empty ``ns`` → wrapped in a GroupAction with PushRosNamespace.
    """
    nodes = [
        Node(
            package='yolo_pcl_cpp_tracker',
            executable='yolo_byte_tracker.py',
            name='yolo_byte_tracker',
            output='screen'),
        Node(
            package='yolo_pcl_cpp_tracker',
            executable='pcl_icp_pose_estimator',
            name='pcl_icp_pose_estimator',
            parameters=[tracker_config],
            output='screen'),
    ]
    ns_clean = ns.strip('/')
    if not ns_clean:
        return nodes
    return [GroupAction([PushRosNamespace(ns_clean)] + nodes)]


def _spawn_nodes(context, *args, **kwargs):
    camera_config = LaunchConfiguration('camera_config').perform(context)
    cfg = load_config(camera_config if camera_config else None)

    tracker_config = config_path('yolo_pcl_cpp_tracker', 'tracker_params.yaml')
    filter_config = config_path('pose_filter_cpp', 'filter_params.yaml')
    associator_config = config_path(
        'cross_camera_associator', 'associator_params.yaml')
    smoother_config = config_path(
        'pose_graph_smoother', 'smoother_params.yaml')

    actions = []

    # Per-camera subtrees
    for cam in cfg.cameras:
        actions.extend(_per_camera_nodes(cam.namespace, tracker_config))

    # Shared fusion / filtering / smoothing (single instance, global topics)
    actions.append(Node(
        package='cross_camera_associator',
        executable='associator_node',
        name='cross_camera_associator',
        parameters=[associator_config, {
            'camera_namespaces': [c.namespace for c in cfg.cameras],
        }],
        output='screen'))

    actions.append(Node(
        package='pose_filter_cpp',
        executable='pose_filter_node',
        name='pose_filter',
        parameters=[filter_config, {
            'use_associated_input': True,
        }],
        output='screen'))

    actions.append(Node(
        package='pose_graph_smoother',
        executable='smoother_node',
        name='pose_graph_smoother',
        parameters=[smoother_config],
        output='screen'))

    return actions


def generate_launch_description():
    return LaunchDescription([
        declare_camera_config_arg(
            description=(
                'Path to camera_config*.yaml (empty = 1-cam fallback)'
            ),
        ),
        OpaqueFunction(function=_spawn_nodes),
    ])
