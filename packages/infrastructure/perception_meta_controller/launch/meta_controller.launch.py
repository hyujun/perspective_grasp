"""Launch perception meta controller node."""

from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    config = os.path.join(
        get_package_share_directory('perception_meta_controller'),
        'config', 'meta_controller_params.yaml')

    return LaunchDescription([
        Node(
            package='perception_meta_controller',
            executable='meta_controller_node',
            name='perception_meta_controller',
            parameters=[config],
            output='screen',
        ),
    ])
