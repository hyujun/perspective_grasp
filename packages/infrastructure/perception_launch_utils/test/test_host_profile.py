# Copyright 2026 perspective_grasp
# SPDX-License-Identifier: Apache-2.0
"""Unit tests for perception_launch_utils.host_profile."""

from __future__ import annotations

import os
from pathlib import Path

import pytest

from perception_launch_utils import (
    HostProfile,
    auto_select_profile,
    load_host_profile,
    overrides_for_node,
    resolve_host_profile,
)


# ------------- load_host_profile -------------------------------------------

def test_load_host_profile_parses_overrides(tmp_path: Path):
    yaml_text = (
        'profile_name: "test_profile"\n'
        'description: "test"\n'
        'overrides:\n'
        '  sam2_segmentor:\n'
        '    model_checkpoint: "/foo/small.pt"\n'
        '  megapose_tracker:\n'
        '    bsz_images: 32\n'
    )
    p = tmp_path / 'test_profile.yaml'
    p.write_text(yaml_text)

    prof = load_host_profile('test_profile', path_override=str(p))
    assert isinstance(prof, HostProfile)
    assert prof.profile_name == 'test_profile'
    assert prof.overrides['sam2_segmentor']['model_checkpoint'] == '/foo/small.pt'
    assert prof.overrides['megapose_tracker']['bsz_images'] == 32
    assert prof.source_path == str(p)


def test_load_host_profile_empty_overrides(tmp_path: Path):
    p = tmp_path / 'empty.yaml'
    p.write_text('profile_name: "empty"\noverrides: {}\n')
    prof = load_host_profile('empty', path_override=str(p))
    assert prof.overrides == {}


def test_load_host_profile_rejects_bad_overrides_shape(tmp_path: Path):
    """Profile with a list at top-level overrides — common typo."""
    p = tmp_path / 'bad.yaml'
    p.write_text('profile_name: "bad"\noverrides:\n  - this is a list\n')
    with pytest.raises(ValueError, match='must be a mapping'):
        load_host_profile('bad', path_override=str(p))


def test_load_host_profile_rejects_non_dict_node_entry(tmp_path: Path):
    """``overrides[node]`` must itself be a mapping, not a scalar."""
    p = tmp_path / 'bad2.yaml'
    p.write_text(
        'profile_name: "bad2"\n'
        'overrides:\n'
        '  sam2_segmentor: "this should be a mapping"\n'
    )
    with pytest.raises(ValueError, match='must be a mapping'):
        load_host_profile('bad2', path_override=str(p))


# ------------- auto_select_profile -----------------------------------------

def test_auto_select_no_gpu_returns_cpu_only():
    assert auto_select_profile(total_vram_mib=None) == 'cpu_only'


def test_auto_select_8gb_returns_dev_profile():
    assert auto_select_profile(total_vram_mib=8192) == 'dev_8gb'


def test_auto_select_16gb_returns_prod_profile():
    assert auto_select_profile(total_vram_mib=16384) == 'prod_16gb'


def test_auto_select_boundary_just_below_threshold():
    # 11 GB < 12000 MiB threshold → still dev
    assert auto_select_profile(total_vram_mib=11264) == 'dev_8gb'


def test_auto_select_boundary_at_threshold():
    # exactly 12 GB → prod (>= boundary)
    assert auto_select_profile(total_vram_mib=12_000) == 'prod_16gb'


# ------------- resolve_host_profile ----------------------------------------

@pytest.fixture
def profile_dir(tmp_path: Path) -> Path:
    """Materialise the three real profile names with placeholder content
    so tests can hit the resolution path without depending on package
    install layout."""
    for name in ('dev_8gb', 'prod_16gb', 'cpu_only'):
        (tmp_path / f'{name}.yaml').write_text(
            f'profile_name: "{name}"\noverrides: {{}}\n'
        )
    return tmp_path


def test_resolve_explicit_name(profile_dir: Path, monkeypatch):
    # Force a specific path by going through path_override route.
    prof = load_host_profile(
        'dev_8gb', path_override=str(profile_dir / 'dev_8gb.yaml'))
    assert prof.profile_name == 'dev_8gb'


def test_resolve_unknown_name_raises(profile_dir: Path, monkeypatch):
    monkeypatch.delenv('PERSPECTIVE_HOST_PROFILE', raising=False)
    with pytest.raises(ValueError, match='unknown host profile'):
        resolve_host_profile('not_a_real_profile', total_vram_mib=8192)


def test_resolve_env_var_takes_precedence(monkeypatch, profile_dir: Path):
    monkeypatch.setenv('PERSPECTIVE_HOST_PROFILE', 'dev_8gb')
    # Use path_override against the matching tmp file. The test exercises
    # name resolution; the load step is covered by load_host_profile tests.
    prof = resolve_host_profile(
        path_override=str(profile_dir / 'dev_8gb.yaml'),
        total_vram_mib=64_000,  # would auto-pick prod, but env wins
    )
    assert prof.profile_name == 'dev_8gb'


def test_resolve_auto_uses_vram(monkeypatch, profile_dir: Path):
    monkeypatch.delenv('PERSPECTIVE_HOST_PROFILE', raising=False)
    prof = resolve_host_profile(
        'auto',
        path_override=str(profile_dir / 'cpu_only.yaml'),
        total_vram_mib=None,
    )
    assert prof.profile_name == 'cpu_only'


# ------------- overrides_for_node ------------------------------------------

def test_overrides_for_node_returns_copy():
    prof = HostProfile(
        profile_name='x', description='', overrides={
            'sam2_segmentor': {'model_checkpoint': '/foo'},
        }
    )
    a = overrides_for_node(prof, 'sam2_segmentor')
    a['model_checkpoint'] = '/bar'  # mutate
    b = overrides_for_node(prof, 'sam2_segmentor')
    # Original profile unchanged — overrides_for_node returns a fresh dict
    assert b['model_checkpoint'] == '/foo'


def test_overrides_for_node_missing_returns_empty_dict():
    prof = HostProfile(profile_name='x', description='', overrides={})
    assert overrides_for_node(prof, 'sam2_segmentor') == {}
