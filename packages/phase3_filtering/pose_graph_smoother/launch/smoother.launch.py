"""Launch pose graph smoother node."""

from launch import LaunchDescription
from launch_ros.actions import LifecycleNode

from perception_launch_utils import config_path


def generate_launch_description():
    return LaunchDescription([
        LifecycleNode(
            package='pose_graph_smoother',
            executable='smoother_node',
            name='pose_graph_smoother',
            namespace='',
            parameters=[config_path(
                'pose_graph_smoother', 'smoother_params.yaml')],
            output='screen',
        ),
    ])
