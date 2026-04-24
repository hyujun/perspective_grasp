from launch import LaunchDescription
from launch_ros.actions import Node

from perception_launch_utils import config_path


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='pose_filter_cpp',
            executable='pose_filter_node',
            name='pose_filter',
            parameters=[config_path('pose_filter_cpp', 'filter_params.yaml')],
            output='screen',
        ),
    ])
