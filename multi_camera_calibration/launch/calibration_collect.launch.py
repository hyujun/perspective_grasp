# Copyright 2026 perspective_grasp
# SPDX-License-Identifier: Apache-2.0

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_dir = get_package_share_directory("multi_camera_calibration")
    default_params = os.path.join(pkg_dir, "config", "calibration_params.yaml")

    return LaunchDescription([
        DeclareLaunchArgument(
            "params_file",
            default_value=default_params,
            description="Path to calibration parameters YAML file",
        ),
        Node(
            package="multi_camera_calibration",
            executable="collect_calibration_data.py",
            name="calibration_data_collector",
            parameters=[LaunchConfiguration("params_file")],
            output="screen",
        ),
    ])
