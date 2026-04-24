# Copyright 2026 perspective_grasp
# SPDX-License-Identifier: Apache-2.0
"""Shared launch-file helpers for every perspective_grasp ROS 2 package.

Re-exports the small public API so callers can write ``from
perception_launch_utils import config_path, fanout_lifecycle_nodes``
instead of importing from submodules.
"""

from .camera_config import (
    CameraSpec,
    PerceptionSystemConfig,
    compose_topic,
    load_cameras,
    load_config,
)
from .fanout import fanout_lifecycle_nodes
from .lifecycle import autostart_lifecycle_actions
from .paths import (
    config_path,
    declare_autostart_arg,
    declare_camera_config_arg,
    declare_params_file_arg,
    share_file,
)

__all__ = [
    # camera_config
    'CameraSpec',
    'PerceptionSystemConfig',
    'compose_topic',
    'load_cameras',
    'load_config',
    # paths
    'config_path',
    'share_file',
    'declare_autostart_arg',
    'declare_camera_config_arg',
    'declare_params_file_arg',
    # lifecycle / fanout
    'autostart_lifecycle_actions',
    'fanout_lifecycle_nodes',
]
