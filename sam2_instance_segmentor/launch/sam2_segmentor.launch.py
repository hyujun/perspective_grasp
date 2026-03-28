"""Launch SAM2 instance segmentor node."""

from launch import LaunchDescription
from launch_ros.actions import LifecycleNode
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    config = os.path.join(
        get_package_share_directory('sam2_instance_segmentor'),
        'config', 'sam2_params.yaml')

    return LaunchDescription([
        LifecycleNode(
            package='sam2_instance_segmentor',
            executable='sam2_segmentor_node',
            name='sam2_segmentor',
            namespace='',
            parameters=[config],
            output='screen',
        ),
    ])
