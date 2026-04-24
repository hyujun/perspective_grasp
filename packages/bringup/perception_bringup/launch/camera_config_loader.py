# Copyright 2026 perspective_grasp
# SPDX-License-Identifier: Apache-2.0
"""Backwards-compat shim for the old importlib-based loader.

The real implementation now lives in the ``perception_launch_utils``
Python package. This shim only exists so any out-of-tree code that
still does::

    importlib.util.spec_from_file_location(
        'cc_loader',
        os.path.join(get_package_share_directory('perception_bringup'),
                     'launch', 'camera_config_loader.py'))

keeps working. New callers should ``from perception_launch_utils import
load_config`` directly — no importlib dance required.
"""

from perception_launch_utils.camera_config import (  # noqa: F401
    CameraSpec,
    PerceptionSystemConfig,
    compose_topic,
    load_cameras,
    load_config,
)
