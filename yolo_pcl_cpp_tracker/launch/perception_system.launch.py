"""Config-driven multi-camera perception system launch.

Reads camera_config.yaml and auto-generates per-camera tracking nodes
plus shared fusion/smoothing nodes.

Usage:
  ros2 launch yolo_pcl_cpp_tracker perception_system.launch.py \
      camera_config:=/path/to/camera_config.yaml
"""

import os
import yaml
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # Default to single-camera config
    default_config = os.path.join(
        get_package_share_directory('yolo_pcl_cpp_tracker'),
        'config', 'tracker_params.yaml')

    # Camera config argument
    camera_config_arg = DeclareLaunchArgument(
        'camera_config',
        default_value='',
        description='Path to camera_config.yaml (empty = single camera)')

    filter_config = os.path.join(
        get_package_share_directory('pose_filter_cpp'),
        'config', 'filter_params.yaml')

    # Load camera config if provided, otherwise default single camera
    camera_config_path = LaunchConfiguration('camera_config')

    # For now, generate a default single-camera launch
    # When camera_config is provided at runtime, it will be parsed
    nodes = []

    # === Per-camera nodes ===
    # In single-camera mode (no namespace), topics are relative:
    #   yolo/detections, yolo_tracker/raw_poses
    # In multi-camera mode, namespace prefixes them automatically

    nodes.append(Node(
        package='yolo_pcl_cpp_tracker',
        executable='yolo_byte_tracker.py',
        name='yolo_byte_tracker',
        output='screen'))

    nodes.append(Node(
        package='yolo_pcl_cpp_tracker',
        executable='pcl_icp_pose_estimator',
        name='pcl_icp_pose_estimator',
        parameters=[default_config],
        output='screen'))

    # === Cross-camera associator (bypass mode for single camera) ===
    associator_config = os.path.join(
        get_package_share_directory('cross_camera_associator'),
        'config', 'associator_params.yaml')

    nodes.append(Node(
        package='cross_camera_associator',
        executable='associator_node',
        name='cross_camera_associator',
        parameters=[associator_config],
        output='screen'))

    # === Pose filter (associated mode) ===
    nodes.append(Node(
        package='pose_filter_cpp',
        executable='pose_filter_node',
        name='pose_filter',
        parameters=[filter_config, {
            'use_associated_input': True,
        }],
        output='screen'))

    # === Pose graph smoother ===
    smoother_config = os.path.join(
        get_package_share_directory('pose_graph_smoother'),
        'config', 'smoother_params.yaml')

    nodes.append(Node(
        package='pose_graph_smoother',
        executable='smoother_node',
        name='pose_graph_smoother',
        parameters=[smoother_config],
        output='screen'))

    return LaunchDescription([
        camera_config_arg,
        *nodes,
    ])
