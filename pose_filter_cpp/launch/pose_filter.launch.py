from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    config = os.path.join(
        get_package_share_directory('pose_filter_cpp'),
        'config', 'filter_params.yaml')

    return LaunchDescription([
        Node(
            package='pose_filter_cpp',
            executable='pose_filter_node',
            name='pose_filter',
            parameters=[config],
            output='screen',
        ),
    ])
