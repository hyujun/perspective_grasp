"""Config-driven multi-camera perception system launch.

Reads ``camera_config*.yaml`` and auto-generates per-camera tracking nodes
plus shared fusion / smoothing / debug nodes.

Usage::

  ros2 launch perception_bringup perception_system.launch.py \\
      camera_config:=/path/to/camera_config.yaml

When ``camera_config`` is empty, falls back to a single root-namespace camera
(identical to ``camera_config_1cam.yaml``).
"""

import importlib.util
import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, PushRosNamespace
from launch.actions import GroupAction
from ament_index_python.packages import get_package_share_directory


def _load_camera_config_loader():
    path = os.path.join(
        get_package_share_directory('perception_bringup'),
        'launch', 'camera_config_loader.py')
    spec = importlib.util.spec_from_file_location(
        'perception_bringup_cc_loader', path)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)  # type: ignore[union-attr]
    return mod


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
    loader = _load_camera_config_loader()
    camera_config = LaunchConfiguration('camera_config').perform(context)
    cfg = loader.load_config(camera_config if camera_config else None)

    tracker_config = os.path.join(
        get_package_share_directory('yolo_pcl_cpp_tracker'),
        'config', 'tracker_params.yaml')
    filter_config = os.path.join(
        get_package_share_directory('pose_filter_cpp'),
        'config', 'filter_params.yaml')
    associator_config = os.path.join(
        get_package_share_directory('cross_camera_associator'),
        'config', 'associator_params.yaml')
    smoother_config = os.path.join(
        get_package_share_directory('pose_graph_smoother'),
        'config', 'smoother_params.yaml')

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
        DeclareLaunchArgument(
            'camera_config',
            default_value='',
            description='Path to camera_config*.yaml (empty = 1-cam fallback)'),
        OpaqueFunction(function=_spawn_nodes),
    ])
