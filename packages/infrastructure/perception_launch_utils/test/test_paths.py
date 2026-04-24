# Copyright 2026 perspective_grasp
# SPDX-License-Identifier: Apache-2.0
"""Unit tests for perception_launch_utils.paths."""

import os
from unittest.mock import patch

from launch.actions import DeclareLaunchArgument

from perception_launch_utils import (
    config_path,
    declare_autostart_arg,
    declare_camera_config_arg,
    declare_params_file_arg,
    share_file,
)


@patch('perception_launch_utils.paths.get_package_share_directory')
def test_config_path_default_filename(mock_share):
    mock_share.return_value = '/install/share/foo'
    assert config_path('foo') == '/install/share/foo/config/foo_params.yaml'
    mock_share.assert_called_once_with('foo')


@patch('perception_launch_utils.paths.get_package_share_directory')
def test_config_path_custom_filename(mock_share):
    mock_share.return_value = '/install/share/foo'
    assert (
        config_path('foo', 'custom.yaml')
        == '/install/share/foo/config/custom.yaml'
    )


@patch('perception_launch_utils.paths.get_package_share_directory')
def test_share_file(mock_share):
    mock_share.return_value = '/install/share/foo'
    assert (
        share_file('foo', 'rviz', 'view.rviz')
        == '/install/share/foo/rviz/view.rviz'
    )


@patch('perception_launch_utils.paths.get_package_share_directory')
def test_declare_params_file_arg_default(mock_share):
    mock_share.return_value = '/install/share/bar'
    arg = declare_params_file_arg('bar')
    assert isinstance(arg, DeclareLaunchArgument)
    assert arg.name == 'params_file'
    assert arg.default_value[0].text == '/install/share/bar/config/bar_params.yaml'


def test_declare_camera_config_arg_default_empty():
    arg = declare_camera_config_arg()
    assert isinstance(arg, DeclareLaunchArgument)
    assert arg.name == 'camera_config'
    assert arg.default_value[0].text == ''


def test_declare_autostart_arg_default_true():
    arg = declare_autostart_arg()
    assert isinstance(arg, DeclareLaunchArgument)
    assert arg.name == 'autostart'
    assert arg.default_value[0].text == 'true'
