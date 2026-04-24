# Copyright 2026 perspective_grasp
# SPDX-License-Identifier: Apache-2.0

from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from perception_launch_utils import declare_params_file_arg


def generate_launch_description():
    return LaunchDescription([
        declare_params_file_arg(
            'multi_camera_calibration', 'calibration_params.yaml',
            description='Path to calibration parameters YAML file',
        ),
        Node(
            package='multi_camera_calibration',
            executable='collect_calibration_data.py',
            name='calibration_data_collector',
            parameters=[LaunchConfiguration('params_file')],
            output='screen',
        ),
    ])
