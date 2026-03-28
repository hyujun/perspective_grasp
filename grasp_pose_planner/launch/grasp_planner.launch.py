"""Launch grasp pose planner node."""

from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    config = os.path.join(
        get_package_share_directory('grasp_pose_planner'),
        'config', 'grasp_planner_params.yaml')

    return LaunchDescription([
        Node(
            package='grasp_pose_planner',
            executable='grasp_planner_node',
            name='grasp_planner',
            parameters=[config],
            output='screen',
        ),
    ])
