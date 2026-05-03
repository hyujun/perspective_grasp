# Copyright 2026 perspective_grasp
# SPDX-License-Identifier: Apache-2.0
"""Host-profile helpers — parameter overrides keyed by host class.

The dev box (RTX 3070 Ti / 8 GB) and the execution PC (RTX A4000+ /
16 GB+) are the same workspace running the same code, but at different
model sizes / batch sizes. This module loads ``host_profile.yaml``
files from ``perception_launch_utils/host_profiles/`` and exposes
their ``overrides`` map so launch files can splat them on top of each
node's default parameter file. The YAMLs ship with this package (not
``perception_bringup``) so Phase 4 Docker containers — which only
install ``perception_launch_utils`` plus their own node — can resolve
profiles without pulling in the full bringup package.

It also provides ``auto_select_profile`` which picks a sensible
profile from ``nvidia-smi`` total VRAM. Anti-pattern (g) in CLAUDE.md
explicitly carves out "the VRAM split is real but only at the param
layer, not in code paths" — that's what this module enforces.
"""

from __future__ import annotations

import os
import re
import shutil
import subprocess
from dataclasses import dataclass, field
from typing import Optional

import yaml

from .paths import share_file


_PROFILE_ENV = 'PERSPECTIVE_HOST_PROFILE'
_PROFILE_DIR_PKG = 'perception_launch_utils'
_PROFILE_DIR_REL = 'host_profiles'
_VALID_PROFILES = ('dev_8gb', 'prod_16gb', 'cpu_only')
# Boundary used by auto-select: GPUs reporting ≥ this many MiB land on
# prod_16gb, otherwise dev_8gb. 12000 ≈ "anything bigger than the 3070
# Ti's 8 GB but smaller than the typical 16 GB target". Adjust if a
# new tier (e.g. 24 GB) becomes worth its own profile.
_PROD_VRAM_MIN_MIB = 12_000


@dataclass(frozen=True)
class HostProfile:
    """Parsed host profile.

    ``overrides`` is the per-node parameter dict. Empty dict (e.g.
    ``prod_16gb``) means "no overrides — use each node's default
    YAML as-is". ``source_path`` is provided for diagnostic logging.
    """

    profile_name: str
    description: str
    overrides: dict[str, dict] = field(default_factory=dict)
    source_path: str = ''


def _profile_path(name: str) -> str:
    """Resolve ``share/perception_launch_utils/host_profiles/<name>.yaml``."""
    return share_file(_PROFILE_DIR_PKG, _PROFILE_DIR_REL, f'{name}.yaml')


def load_host_profile(
    name: str,
    *,
    path_override: Optional[str] = None,
) -> HostProfile:
    """Load a profile YAML by name (or absolute path) and parse it.

    ``path_override`` is the test seam — when set, the file is read
    directly from that path instead of going through
    :func:`get_package_share_directory`.
    """
    path = path_override or _profile_path(name)
    with open(path, 'r') as f:
        raw = yaml.safe_load(f) or {}
    overrides = raw.get('overrides') or {}
    if not isinstance(overrides, dict):
        raise ValueError(
            f'host profile {path}: "overrides" must be a mapping, '
            f'got {type(overrides).__name__}'
        )
    # Each per-node entry must itself be a mapping. Catch the typo of
    # writing a flat list at the top level rather than letting it blow
    # up later when launch tries to iterate ``.items()``.
    for node_name, params in overrides.items():
        if not isinstance(params, dict):
            raise ValueError(
                f'host profile {path}: overrides["{node_name}"] must be a '
                f'mapping, got {type(params).__name__}'
            )
    return HostProfile(
        profile_name=str(raw.get('profile_name', name)),
        description=str(raw.get('description', '')),
        overrides=overrides,
        source_path=path,
    )


def _probe_total_vram_mib() -> Optional[int]:
    """Sum all visible GPUs' total VRAM in MiB, or ``None`` if no GPU.

    Uses ``nvidia-smi --query-gpu=memory.total --format=csv,noheader,nounits``
    which prints one integer per line in MiB.
    """
    if not shutil.which('nvidia-smi'):
        return None
    try:
        out = subprocess.check_output(
            ['nvidia-smi', '--query-gpu=memory.total',
             '--format=csv,noheader,nounits'],
            text=True, timeout=4,
        )
    except Exception:  # noqa: BLE001
        return None
    total = 0
    for line in out.splitlines():
        m = re.search(r'\d+', line)
        if m:
            total += int(m.group(0))
    return total or None


def auto_select_profile(
    *,
    total_vram_mib: Optional[int] = None,
) -> str:
    """Pick a profile name from observed GPU VRAM.

    The argument is a test seam — production callers pass nothing and
    we probe ``nvidia-smi``. Heuristic:

    * No GPU detectable                          → ``cpu_only``
    * Total VRAM < :data:`_PROD_VRAM_MIN_MIB`     → ``dev_8gb``
    * Otherwise                                   → ``prod_16gb``
    """
    vram = total_vram_mib if total_vram_mib is not None else _probe_total_vram_mib()
    if vram is None:
        return 'cpu_only'
    if vram < _PROD_VRAM_MIN_MIB:
        return 'dev_8gb'
    return 'prod_16gb'


def resolve_host_profile(
    requested: Optional[str] = None,
    *,
    path_override: Optional[str] = None,
    total_vram_mib: Optional[int] = None,
) -> HostProfile:
    """Resolve ``requested`` (or env / auto) to a parsed :class:`HostProfile`.

    Priority: explicit argument → ``PERSPECTIVE_HOST_PROFILE`` env var →
    ``auto`` (VRAM-based selection). An unknown name raises
    ``ValueError`` so launch fails loudly rather than silently picking
    a default.
    """
    if not requested:
        requested = os.environ.get(_PROFILE_ENV, 'auto')
    requested = requested.strip()
    if requested == 'auto':
        requested = auto_select_profile(total_vram_mib=total_vram_mib)
    if requested not in _VALID_PROFILES:
        raise ValueError(
            f'unknown host profile {requested!r}; valid: '
            f'{", ".join(_VALID_PROFILES)}'
        )
    return load_host_profile(requested, path_override=path_override)


# ---------------------------------------------------------------------------
# Launch integration
# ---------------------------------------------------------------------------


def declare_host_profile_arg():
    """``DeclareLaunchArgument`` for the standard ``host_profile`` arg.

    Default ``auto`` triggers VRAM-based selection at launch time. The
    arg is kept separate from ``camera_config`` because the two answer
    different questions (host *capability* vs camera *count*).
    """
    from launch.actions import DeclareLaunchArgument

    return DeclareLaunchArgument(
        'host_profile',
        default_value='auto',
        description=(
            'Host profile name (auto / dev_8gb / prod_16gb / cpu_only). '
            'auto picks via nvidia-smi VRAM. Overrides also via '
            'PERSPECTIVE_HOST_PROFILE env var.'
        ),
    )


def overrides_for_node(profile: HostProfile, node_name: str) -> dict:
    """Per-node override slice ready to splat into ``parameters=[...]``.

    Returns ``{}`` for nodes the profile doesn't mention — splatting an
    empty dict into ``parameters`` is a safe no-op in launch_ros, so
    callers never need to special-case profiles that don't touch their
    node.
    """
    return dict(profile.overrides.get(node_name, {}))
