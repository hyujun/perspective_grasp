from launch import LaunchDescription
from launch_ros.actions import Node

from perception_launch_utils import config_path


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='cross_camera_associator',
            executable='associator_node',
            name='cross_camera_associator',
            parameters=[config_path(
                'cross_camera_associator', 'associator_params.yaml')],
            output='screen',
        ),
    ])
