"""Launch perception debug visualizer node."""

from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    config = os.path.join(
        get_package_share_directory('perception_debug_visualizer'),
        'config', 'visualizer_params.yaml')

    return LaunchDescription([
        Node(
            package='perception_debug_visualizer',
            executable='visualizer_node',
            name='perception_debug_visualizer',
            parameters=[config],
            output='screen',
        ),
    ])
