import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    config = os.path.join(
        get_package_share_directory('cross_camera_associator'),
        'config',
        'associator_params.yaml',
    )

    return LaunchDescription([
        Node(
            package='cross_camera_associator',
            executable='associator_node',
            name='cross_camera_associator',
            parameters=[config],
            output='screen',
        ),
    ])
