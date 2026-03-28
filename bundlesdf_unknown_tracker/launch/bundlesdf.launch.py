"""Launch BundleSDF unknown object tracker node."""

from launch import LaunchDescription
from launch_ros.actions import LifecycleNode
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    config = os.path.join(
        get_package_share_directory('bundlesdf_unknown_tracker'),
        'config', 'bundlesdf_params.yaml')

    return LaunchDescription([
        LifecycleNode(
            package='bundlesdf_unknown_tracker',
            executable='bundlesdf_node',
            name='bundlesdf_tracker',
            namespace='',
            parameters=[config],
            output='screen',
        ),
    ])
