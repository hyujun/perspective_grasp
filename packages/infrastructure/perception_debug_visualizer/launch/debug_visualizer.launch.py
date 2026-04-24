"""Launch perception debug visualizer node.

Args:
  camera_config:  Path to perception_bringup camera_config*.yaml. When set, the
                  yaml's camera list drives the visualizer's per-camera
                  subscriptions. Empty → single-camera (root-namespace) mode.
  active_camera:  Index into the camera list to display on /debug/image.
  gui:            If true, launch rqt_image_view showing /debug/image.
  rviz:           If true, launch RViz2 with the bundled debug preset.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from perception_launch_utils import (
    config_path,
    declare_camera_config_arg,
    load_config,
    share_file,
)


def _spawn_nodes(context, *args, **kwargs):
    default_params = config_path(
        'perception_debug_visualizer', 'visualizer_params.yaml')
    rviz_preset = share_file(
        'perception_debug_visualizer', 'rviz', 'debug_view.rviz')

    camera_config = LaunchConfiguration('camera_config').perform(context)
    active_camera = int(LaunchConfiguration('active_camera').perform(context))
    gui = LaunchConfiguration('gui').perform(context).lower() == 'true'
    rviz = LaunchConfiguration('rviz').perform(context).lower() == 'true'

    cfg = load_config(camera_config if camera_config else None)
    namespaces = [c.namespace for c in cfg.cameras]
    active_idx = max(0, min(active_camera, len(namespaces) - 1))

    node_params = [
        default_params,
        {
            'camera_namespaces': namespaces,
            'active_camera_index': active_idx,
        },
    ]

    actions = [
        Node(
            package='perception_debug_visualizer',
            executable='visualizer_node',
            name='perception_debug_visualizer',
            parameters=node_params,
            output='screen',
        ),
    ]

    if gui:
        actions.append(Node(
            package='rqt_image_view',
            executable='rqt_image_view',
            name='debug_image_view',
            arguments=['/debug/image'],
            output='screen',
        ))

    if rviz:
        actions.append(Node(
            package='rviz2',
            executable='rviz2',
            name='perception_debug_rviz',
            arguments=['-d', rviz_preset],
            output='screen',
        ))

    return actions


def generate_launch_description():
    return LaunchDescription([
        declare_camera_config_arg(
            description=(
                'Path to camera_config*.yaml. Empty = single root-namespace camera.'
            ),
        ),
        DeclareLaunchArgument(
            'active_camera',
            default_value='0',
            description='Index (into yaml cameras list) to overlay on /debug/image.'),
        DeclareLaunchArgument(
            'gui',
            default_value='false',
            description='If true, launch rqt_image_view on /debug/image.'),
        DeclareLaunchArgument(
            'rviz',
            default_value='false',
            description='If true, launch RViz2 with the bundled debug preset.'),
        OpaqueFunction(function=_spawn_nodes),
    ])
