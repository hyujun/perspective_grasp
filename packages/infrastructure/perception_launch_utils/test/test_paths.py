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
    repo_root,
    share_file,
    workspace_models_dir,
    workspace_runtime_outputs_dir,
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


# --- workspace path helpers ------------------------------------------------


@patch.dict(os.environ, {'PERSPECTIVE_GRASP_REPO_ROOT': '/custom/repo'},
            clear=False)
def test_repo_root_env_override():
    assert repo_root() == '/custom/repo'


@patch('perception_launch_utils.paths.get_package_share_directory')
def test_repo_root_walks_up_from_share(mock_share, monkeypatch):
    monkeypatch.delenv('PERSPECTIVE_GRASP_REPO_ROOT', raising=False)
    mock_share.return_value = '/ws/install/perception_bringup/share/perception_bringup'
    assert repo_root() == '/ws/src/perspective_grasp'


@patch.dict(os.environ, {'PERSPECTIVE_GRASP_MODELS_DIR': '/elsewhere/models'},
            clear=False)
def test_workspace_models_dir_env_override():
    assert workspace_models_dir() == '/elsewhere/models'


@patch('perception_launch_utils.paths.get_package_share_directory')
def test_workspace_models_dir_default(mock_share, monkeypatch):
    monkeypatch.delenv('PERSPECTIVE_GRASP_REPO_ROOT', raising=False)
    monkeypatch.delenv('PERSPECTIVE_GRASP_MODELS_DIR', raising=False)
    mock_share.return_value = '/ws/install/perception_bringup/share/perception_bringup'
    assert workspace_models_dir() == '/ws/src/perspective_grasp/models'


def test_workspace_runtime_outputs_dir_env_override(tmp_path, monkeypatch):
    monkeypatch.setenv('PERSPECTIVE_GRASP_RUNTIME_OUTPUTS_DIR', str(tmp_path))
    out = workspace_runtime_outputs_dir('calibration')
    assert out == str(tmp_path / 'calibration')
    assert (tmp_path / 'calibration').is_dir()


def test_workspace_runtime_outputs_dir_no_subdir(tmp_path, monkeypatch):
    monkeypatch.setenv('PERSPECTIVE_GRASP_RUNTIME_OUTPUTS_DIR', str(tmp_path))
    out = workspace_runtime_outputs_dir()
    assert out == str(tmp_path)
    assert tmp_path.is_dir()


def test_workspace_runtime_outputs_dir_skip_create(tmp_path, monkeypatch):
    target_root = tmp_path / 'does-not-exist'
    monkeypatch.setenv(
        'PERSPECTIVE_GRASP_RUNTIME_OUTPUTS_DIR', str(target_root))
    out = workspace_runtime_outputs_dir('foo', create=False)
    assert out == str(target_root / 'foo')
    assert not target_root.exists()


@patch('perception_launch_utils.paths.get_package_share_directory')
def test_workspace_runtime_outputs_dir_default_layout(
        mock_share, tmp_path, monkeypatch):
    # Build a fake colcon install layout under tmp_path so the walk-up resolves
    # to a writable repo root.
    ws = tmp_path / 'ws'
    repo = ws / 'src' / 'perspective_grasp'
    repo.mkdir(parents=True)
    mock_share.return_value = str(
        ws / 'install' / 'perception_bringup' / 'share' / 'perception_bringup')
    monkeypatch.delenv('PERSPECTIVE_GRASP_REPO_ROOT', raising=False)
    monkeypatch.delenv('PERSPECTIVE_GRASP_RUNTIME_OUTPUTS_DIR', raising=False)
    out = workspace_runtime_outputs_dir('calibration')
    expected = repo / 'runtime_outputs' / 'calibration'
    assert out == str(expected)
    assert expected.is_dir()
