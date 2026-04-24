# Copyright 2026 perspective_grasp
# SPDX-License-Identifier: Apache-2.0
"""Typed loader for ``camera_config*.yaml`` files.

Supersedes ``perception_bringup/launch/camera_config_loader.py``. Installed
as a regular Python module, so launch files can ``from
perception_launch_utils import load_config`` instead of the old importlib
dance.
"""

from __future__ import annotations

import os
from dataclasses import dataclass, field
from typing import Any, Dict, List, Optional

import yaml


@dataclass
class CameraSpec:
    """Single camera entry parsed from ``camera_config*.yaml``."""

    id: int
    namespace: str          # "" for the root (1-cam) case
    type: str               # "eye_in_hand" or "eye_to_hand"
    frame_id: str
    serial: str = ""
    role: str = ""
    resolution: str = ""
    fps: int = 30
    sync_mode: int = 0
    mount_position: str = ""
    ee_frame: str = ""
    static_tf: Optional[Dict[str, float]] = None
    raw: Dict[str, Any] = field(default_factory=dict)

    @property
    def ns_clean(self) -> str:
        """Namespace stripped of leading/trailing slashes ("" stays "")."""
        return self.namespace.strip('/')


@dataclass
class PerceptionSystemConfig:
    """Full ``perception_system`` block parsed from a config yaml."""

    base_frame: str
    cameras: List[CameraSpec]
    association: Dict[str, Any] = field(default_factory=dict)
    merge: Dict[str, Any] = field(default_factory=dict)
    raw: Dict[str, Any] = field(default_factory=dict)

    @property
    def namespaces(self) -> List[str]:
        return [c.namespace for c in self.cameras]


def _default_single_camera() -> List[CameraSpec]:
    return [
        CameraSpec(
            id=0,
            namespace='',
            type='eye_in_hand',
            frame_id='camera_color_optical_frame',
            role='primary',
        ),
    ]


def load_config(path: Optional[str]) -> PerceptionSystemConfig:
    """Load a ``camera_config*.yaml``. Empty/missing path → 1-cam fallback."""
    if not path or not os.path.isfile(path):
        return PerceptionSystemConfig(
            base_frame='ur5e_base_link',
            cameras=_default_single_camera(),
        )

    with open(path, 'r') as f:
        data = yaml.safe_load(f) or {}

    params = (
        data.get('perception_system', {}).get('ros__parameters', {})
    )
    cameras_raw = params.get('cameras', [])
    cameras = [
        CameraSpec(
            id=int(c.get('id', i)),
            namespace=str(c.get('namespace', '')),
            type=str(c.get('type', 'eye_in_hand')),
            frame_id=str(c.get('frame_id', 'camera_color_optical_frame')),
            serial=str(c.get('serial', '')),
            role=str(c.get('role', '')),
            resolution=str(c.get('resolution', '')),
            fps=int(c.get('fps', 30)),
            sync_mode=int(c.get('sync_mode', 0)),
            mount_position=str(c.get('mount_position', '')),
            ee_frame=str(c.get('ee_frame', '')),
            static_tf=c.get('static_tf'),
            raw=c,
        )
        for i, c in enumerate(cameras_raw)
    ] or _default_single_camera()

    return PerceptionSystemConfig(
        base_frame=str(params.get('base_frame', 'ur5e_base_link')),
        cameras=cameras,
        association=params.get('association', {}) or {},
        merge=params.get('merge', {}) or {},
        raw=params,
    )


def load_cameras(path: Optional[str]) -> List[CameraSpec]:
    """Convenience wrapper returning just the camera list."""
    return load_config(path).cameras


def compose_topic(namespace: str, suffix: str) -> str:
    """Join a camera namespace with a topic suffix.

    ``namespace=""``     + ``suffix="yolo/detections"`` → ``/yolo/detections``
    ``namespace="/cam0"`` + ``suffix="yolo/detections"`` → ``/cam0/yolo/detections``
    """
    ns = namespace.strip('/')
    tail = suffix.strip('/')
    return f'/{ns}/{tail}' if ns else f'/{tail}'
