"""Launch grasp pose planner node."""

from launch import LaunchDescription
from launch_ros.actions import Node

from perception_launch_utils import config_path


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='grasp_pose_planner',
            executable='grasp_planner_node',
            name='grasp_planner',
            parameters=[config_path(
                'grasp_pose_planner', 'grasp_planner_params.yaml')],
            output='screen',
        ),
    ])
