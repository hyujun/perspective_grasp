"""Launch FoundationPose tracker node."""

from launch import LaunchDescription
from launch_ros.actions import LifecycleNode
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    config = os.path.join(
        get_package_share_directory('isaac_foundationpose_tracker'),
        'config', 'foundationpose_params.yaml')

    return LaunchDescription([
        LifecycleNode(
            package='isaac_foundationpose_tracker',
            executable='foundationpose_node',
            name='foundationpose_tracker',
            namespace='',
            parameters=[config],
            output='screen',
        ),
    ])
