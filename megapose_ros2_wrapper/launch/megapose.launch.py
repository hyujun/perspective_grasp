"""Launch MegaPose tracker node."""

from launch import LaunchDescription
from launch_ros.actions import LifecycleNode
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    config = os.path.join(
        get_package_share_directory('megapose_ros2_wrapper'),
        'config', 'megapose_params.yaml')

    return LaunchDescription([
        LifecycleNode(
            package='megapose_ros2_wrapper',
            executable='megapose_node',
            name='megapose_tracker',
            namespace='',
            parameters=[config],
            output='screen',
        ),
    ])
