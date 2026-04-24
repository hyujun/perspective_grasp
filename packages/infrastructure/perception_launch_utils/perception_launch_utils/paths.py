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
