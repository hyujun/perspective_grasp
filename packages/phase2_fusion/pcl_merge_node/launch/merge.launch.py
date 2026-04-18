import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    config = os.path.join(
        get_package_share_directory('pcl_merge_node'),
        'config',
        'merge_params.yaml',
    )

    merge_node = Node(
        package='pcl_merge_node',
        executable='merge_node',
        name='pcl_merge_node',
        parameters=[config],
        output='screen',
    )

    return LaunchDescription([merge_node])
