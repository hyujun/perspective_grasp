# Copyright 2026 perspective_grasp
# SPDX-License-Identifier: Apache-2.0
"""Unit tests for perception_launch_utils.paths."""

import os
from unittest.mock import patch

import pytest
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
from perception_launch_utils.paths import _resolve_repo_root_from_workspace


@pytest.fixture(autouse=True)
def _clear_repo_root_cache():
    """Each test gets a fresh fallback resolution — the lru_cache on the
    workspace search would otherwise leak state between tests."""
    _resolve_repo_root_from_workspace.cache_clear()
    yield
    _resolve_repo_root_from_workspace.cache_clear()


def _make_fake_install(ws, anchor_pkg, repo_dir, sub_path=('packages',)):
    """Create a colcon-shaped install layout plus a source tree containing
    ``anchor_pkg``'s package.xml at ``<repo_dir>/<sub_path>/<anchor_pkg>``.

    Returns the share dir path that ``get_package_share_directory`` should
    mock to.
    """
    src_pkg = repo_dir.joinpath(*sub_path, anchor_pkg)
    src_pkg.mkdir(parents=True)
    (src_pkg / 'package.xml').write_text(
        f'<?xml version="1.0"?><package format="3"><name>{anchor_pkg}</name>'
        '</package>'
    )
    share = ws / 'install' / anchor_pkg / 'share' / anchor_pkg
    share.mkdir(parents=True)
    return str(share)


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
def test_repo_root_resolves_via_package_xml(mock_share, tmp_path, monkeypatch):
    monkeypatch.delenv('PERSPECTIVE_GRASP_REPO_ROOT', raising=False)
    ws = tmp_path / 'ws'
    repo = ws / 'src' / 'perspective_grasp'
    mock_share.return_value = _make_fake_install(ws, 'perception_bringup', repo)
    assert repo_root() == str(repo)


@patch('perception_launch_utils.paths.get_package_share_directory')
def test_repo_root_works_with_renamed_folder(mock_share, tmp_path, monkeypatch):
    """Folder name need not be 'perspective_grasp' anymore."""
    monkeypatch.delenv('PERSPECTIVE_GRASP_REPO_ROOT', raising=False)
    ws = tmp_path / 'ws'
    repo = ws / 'src' / 'totally_different_name'
    mock_share.return_value = _make_fake_install(ws, 'perception_bringup', repo)
    assert repo_root() == str(repo)


@patch('perception_launch_utils.paths.get_package_share_directory')
def test_repo_root_walks_up_through_nested_groups(
        mock_share, tmp_path, monkeypatch):
    """Anchor's package.xml may sit several levels under packages/<group>/."""
    monkeypatch.delenv('PERSPECTIVE_GRASP_REPO_ROOT', raising=False)
    ws = tmp_path / 'ws'
    repo = ws / 'src' / 'my_repo'
    mock_share.return_value = _make_fake_install(
        ws, 'perception_bringup', repo, sub_path=('packages', 'bringup'))
    assert repo_root() == str(repo)


@patch('perception_launch_utils.paths.get_package_share_directory')
def test_repo_root_raises_when_anchor_missing(mock_share, tmp_path,
                                              monkeypatch):
    """If no matching package.xml exists under <ws>/src, raise with the
    env-var hint."""
    monkeypatch.delenv('PERSPECTIVE_GRASP_REPO_ROOT', raising=False)
    ws = tmp_path / 'ws'
    # Install tree exists; src tree is empty (simulates a deployment box
    # where the source was wiped or relocated outside the workspace).
    (ws / 'install' / 'perception_bringup' / 'share' /
     'perception_bringup').mkdir(parents=True)
    (ws / 'src').mkdir(parents=True)
    mock_share.return_value = str(
        ws / 'install' / 'perception_bringup' / 'share' / 'perception_bringup')
    with pytest.raises(FileNotFoundError, match='PERSPECTIVE_GRASP_REPO_ROOT'):
        repo_root()


@patch('perception_launch_utils.paths.get_package_share_directory')
def test_repo_root_raises_when_no_packages_marker(
        mock_share, tmp_path, monkeypatch):
    """Anchor exists but no ancestor has a `packages/` marker — bail out."""
    monkeypatch.delenv('PERSPECTIVE_GRASP_REPO_ROOT', raising=False)
    ws = tmp_path / 'ws'
    # Place package.xml directly under src/, without the packages/ marker.
    src_pkg = ws / 'src' / 'perception_bringup'
    src_pkg.mkdir(parents=True)
    (src_pkg / 'package.xml').write_text(
        '<?xml version="1.0"?><package format="3">'
        '<name>perception_bringup</name></package>'
    )
    share = ws / 'install' / 'perception_bringup' / 'share' / 'perception_bringup'
    share.mkdir(parents=True)
    mock_share.return_value = str(share)
    with pytest.raises(FileNotFoundError, match='PERSPECTIVE_GRASP_REPO_ROOT'):
        repo_root()


@patch.dict(os.environ, {'PERSPECTIVE_GRASP_MODELS_DIR': '/elsewhere/models'},
            clear=False)
def test_workspace_models_dir_env_override():
    assert workspace_models_dir() == '/elsewhere/models'


@patch('perception_launch_utils.paths.get_package_share_directory')
def test_workspace_models_dir_default(mock_share, tmp_path, monkeypatch):
    monkeypatch.delenv('PERSPECTIVE_GRASP_REPO_ROOT', raising=False)
    monkeypatch.delenv('PERSPECTIVE_GRASP_MODELS_DIR', raising=False)
    ws = tmp_path / 'ws'
    repo = ws / 'src' / 'perspective_grasp'
    mock_share.return_value = _make_fake_install(ws, 'perception_bringup', repo)
    assert workspace_models_dir() == str(repo / 'models')


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
    ws = tmp_path / 'ws'
    repo = ws / 'src' / 'perspective_grasp'
    mock_share.return_value = _make_fake_install(ws, 'perception_bringup', repo)
    monkeypatch.delenv('PERSPECTIVE_GRASP_REPO_ROOT', raising=False)
    monkeypatch.delenv('PERSPECTIVE_GRASP_RUNTIME_OUTPUTS_DIR', raising=False)
    out = workspace_runtime_outputs_dir('calibration')
    expected = repo / 'runtime_outputs' / 'calibration'
    assert out == str(expected)
    assert expected.is_dir()
