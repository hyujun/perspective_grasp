"""Launch perception meta controller node."""

from launch import LaunchDescription
from launch_ros.actions import Node

from perception_launch_utils import config_path


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='perception_meta_controller',
            executable='meta_controller_node',
            name='perception_meta_controller',
            parameters=[config_path(
                'perception_meta_controller', 'meta_controller_params.yaml')],
            output='screen',
        ),
    ])
