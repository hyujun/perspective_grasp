# Copyright 2026 perspective_grasp
# SPDX-License-Identifier: Apache-2.0
"""Config-path helpers shared by every perspective_grasp launch file.

Collapses the repeated 5-line ``os.path.join(get_package_share_directory(...),
'config', '<pkg>_params.yaml')`` block into a single call.
"""

from __future__ import annotations

import os
from typing import Optional

from ament_index_python.packages import get_package_share_directory
from launch.actions import DeclareLaunchArgument

# Workspace layout: <ws>/install/<pkg>/share/<pkg> → walk up 4 levels → <ws>.
# The repo lives at <ws>/src/perspective_grasp.
_REPO_REL_FROM_WS = ('src', 'perspective_grasp')
_MODELS_SUBDIR = 'models'
_RUNTIME_OUTPUTS_SUBDIR = 'runtime_outputs'

_ENV_REPO_ROOT = 'PERSPECTIVE_GRASP_REPO_ROOT'
_ENV_MODELS_DIR = 'PERSPECTIVE_GRASP_MODELS_DIR'
_ENV_RUNTIME_OUTPUTS_DIR = 'PERSPECTIVE_GRASP_RUNTIME_OUTPUTS_DIR'


def repo_root(anchor_pkg: str = 'perception_bringup') -> str:
    """Absolute path to the perspective_grasp source tree.

    Resolution order:
    1. ``$PERSPECTIVE_GRASP_REPO_ROOT`` if set.
    2. Walk up 4 levels from ``share/<anchor_pkg>`` (the standard colcon
       ``install/<pkg>/share/<pkg>`` layout) to the colcon workspace, then
       descend into ``src/perspective_grasp``.

    The anchor package must be installed (i.e., ``colcon build`` ran for it).
    Defaults to ``perception_bringup`` because every system launch already
    depends on it.
    """
    env = os.environ.get(_ENV_REPO_ROOT)
    if env:
        return env
    share = get_package_share_directory(anchor_pkg)
    ws_root = os.path.abspath(os.path.join(share, '..', '..', '..', '..'))
    return os.path.join(ws_root, *_REPO_REL_FROM_WS)


def workspace_models_dir(anchor_pkg: str = 'perception_bringup') -> str:
    """Absolute path to ``<repo>/models``.

    Override with ``$PERSPECTIVE_GRASP_MODELS_DIR``. Pre-positioned weights,
    meshes, and datasets live here; auto-downloaded weights (e.g. YOLO's
    ``yolov8n.pt``) also land here when nodes ``chdir`` to it before load.
    """
    env = os.environ.get(_ENV_MODELS_DIR)
    if env:
        return env
    return os.path.join(repo_root(anchor_pkg), _MODELS_SUBDIR)


def workspace_runtime_outputs_dir(
    subdir: str = '',
    *,
    anchor_pkg: str = 'perception_bringup',
    create: bool = True,
) -> str:
    """Absolute path to ``<repo>/runtime_outputs[/<subdir>]``.

    Override the root with ``$PERSPECTIVE_GRASP_RUNTIME_OUTPUTS_DIR``. Holds
    runtime-generated artifacts (calibration results, BundleSDF dumps, debug
    recordings) — distinct from ``models/`` which is for input assets.

    When ``create=True`` (default), the directory is created if missing so
    callers can rely on it existing before passing it as a node parameter.
    """
    env = os.environ.get(_ENV_RUNTIME_OUTPUTS_DIR)
    if env:
        root = env
    else:
        root = os.path.join(repo_root(anchor_pkg), _RUNTIME_OUTPUTS_SUBDIR)
    target = os.path.join(root, subdir) if subdir else root
    if create:
        os.makedirs(target, exist_ok=True)
    return target


def config_path(pkg: str, filename: Optional[str] = None) -> str:
    """Absolute path to ``share/<pkg>/config/<filename>``.

    ``filename=None`` defaults to ``<pkg>_params.yaml`` — the convention
    used by every node in this workspace.
    """
    name = filename or f'{pkg}_params.yaml'
    return os.path.join(get_package_share_directory(pkg), 'config', name)


def share_file(pkg: str, *relative_parts: str) -> str:
    """Absolute path to ``share/<pkg>/<relative_parts...>``.

    Generalization of ``config_path`` for non-``config/`` resources
    (``launch/``, ``rviz/``, ``meshes/`` ...).
    """
    return os.path.join(get_package_share_directory(pkg), *relative_parts)


def declare_params_file_arg(
    pkg: str,
    filename: Optional[str] = None,
    *,
    arg_name: str = 'params_file',
    description: Optional[str] = None,
) -> DeclareLaunchArgument:
    """``DeclareLaunchArgument`` for a node's YAML params file.

    Default value is ``config_path(pkg, filename)`` so ``ros2 launch``
    users can override with ``params_file:=/custom/path.yaml`` without
    editing the launch file.
    """
    default = config_path(pkg, filename)
    resolved_name = filename or f'{pkg}_params.yaml'
    return DeclareLaunchArgument(
        arg_name,
        default_value=default,
        description=description or f'Path to {resolved_name}',
    )


def declare_camera_config_arg(
    *,
    arg_name: str = 'camera_config',
    default_value: str = '',
    description: Optional[str] = None,
) -> DeclareLaunchArgument:
    """Standard ``camera_config`` arg (default empty → 1-cam fallback)."""
    return DeclareLaunchArgument(
        arg_name,
        default_value=default_value,
        description=description or (
            'Path to camera_config*.yaml. When empty, spawns a single node '
            'with default (non-namespaced) topics. Otherwise spawns one '
            'node per entry in perception_system.cameras.'
        ),
    )


def declare_autostart_arg(
    *,
    arg_name: str = 'autostart',
    default_value: str = 'true',
    description: Optional[str] = None,
) -> DeclareLaunchArgument:
    """Standard ``autostart`` arg for LifecycleNode-based launches."""
    return DeclareLaunchArgument(
        arg_name,
        default_value=default_value,
        description=description or (
            'Auto-drive the LifecycleNode(s) to ACTIVE on launch. '
            'Set to false to manage the lifecycle externally.'
        ),
    )
