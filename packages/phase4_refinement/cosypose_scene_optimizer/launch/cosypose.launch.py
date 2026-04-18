"""Launch CosyPose scene optimizer node."""

from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    config = os.path.join(
        get_package_share_directory('cosypose_scene_optimizer'),
        'config', 'cosypose_params.yaml')

    return LaunchDescription([
        Node(
            package='cosypose_scene_optimizer',
            executable='cosypose_node',
            name='cosypose_optimizer',
            parameters=[config],
            output='screen',
        ),
    ])
