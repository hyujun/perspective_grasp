from launch import LaunchDescription
from launch_ros.actions import Node

from perception_launch_utils import config_path


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='pcl_merge_node',
            executable='merge_node',
            name='pcl_merge_node',
            parameters=[config_path('pcl_merge_node', 'merge_params.yaml')],
            output='screen',
        ),
    ])
