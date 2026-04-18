"""Launch pose graph smoother node."""

from launch import LaunchDescription
from launch_ros.actions import LifecycleNode
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    config = os.path.join(
        get_package_share_directory('pose_graph_smoother'),
        'config', 'smoother_params.yaml')

    return LaunchDescription([
        LifecycleNode(
            package='pose_graph_smoother',
            executable='smoother_node',
            name='pose_graph_smoother',
            namespace='',
            parameters=[config],
            output='screen',
        ),
    ])
