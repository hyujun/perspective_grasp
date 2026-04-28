# Copyright 2026 perspective_grasp
# SPDX-License-Identifier: Apache-2.0
"""Unit tests for perception_launch_utils.preflight.

The launch-action wrapper is exercised separately in launch tests; these
focus on :func:`check_host_environment` so we can run them with plain
pytest without ROS sourced.
"""

from __future__ import annotations

from types import SimpleNamespace

from perception_launch_utils import (
    PreflightReport,
    check_host_environment,
)
from perception_launch_utils.preflight import _parse_nvidia_smi


# Realistic nvidia-smi header, abridged. Driver / CUDA Version / GPU model
# all appear in the same shape across modern releases (2022+).
_NVSMI_TEMPLATE = """\
{header}
+-----------------------------------------------------------------------------+
| NVIDIA-SMI {drv}            Driver Version: {drv}      CUDA Version: {cuda} |
|-------------------------------+----------------------+----------------------+
| GPU  Name                  Persistence-M | Bus-Id        Disp.A | Volatile  |
|   0  {gpu}                       Off    | 00000000:01:00.0 Off |       N/A |
+-----------------------------------------------------------------------------+
"""


def _nvsmi(driver: str, cuda: str, gpu: str = 'NVIDIA RTX A4000') -> str:
    return _NVSMI_TEMPLATE.format(header='', drv=driver, cuda=cuda, gpu=gpu)


def _fake_torch(*, version: str, cuda: str, available: bool):
    """Stand-in torch module."""
    return SimpleNamespace(
        __version__=version,
        version=SimpleNamespace(cuda=cuda),
        cuda=SimpleNamespace(is_available=lambda: available),
    )


# -- _parse_nvidia_smi -------------------------------------------------------

def test_parse_nvidia_smi_pulls_driver_cuda_gpu():
    drv, cuda, gpu = _parse_nvidia_smi(_nvsmi('570.211.01', '12.8'))
    assert drv == '570.211.01'
    assert cuda == '12.8'
    assert 'NVIDIA RTX A4000' in (gpu or '')


def test_parse_nvidia_smi_handles_missing_fields():
    drv, cuda, gpu = _parse_nvidia_smi('garbage output\nno fields here')
    assert drv is None
    assert cuda is None
    assert gpu is None


# -- check_host_environment --------------------------------------------------

def test_skip_returns_skipped_report():
    rep = check_host_environment(skip=True)
    assert isinstance(rep, PreflightReport)
    assert rep.skipped is True
    assert rep.ok is True
    assert 'skipped' in rep.render()


def test_matching_versions_clean_report():
    rep = check_host_environment(
        torch_mod=_fake_torch(version='2.6.0', cuda='12.6', available=True),
        nvidia_smi_output=_nvsmi('560.94', '12.6'),
    )
    assert rep.ok is True
    assert rep.warnings == []
    assert rep.driver_cuda == '12.6'
    assert rep.torch_cuda == '12.6'
    assert rep.cuda_available is True


def test_torch_cuda_higher_than_driver_warns():
    """The motivating case: cu130 wheel on a CUDA-12.8 / driver 570 box."""
    rep = check_host_environment(
        torch_mod=_fake_torch(version='2.11.0+cu130', cuda='13.0', available=False),
        nvidia_smi_output=_nvsmi('570.211.01', '12.8'),
    )
    assert rep.ok is True  # warning, not error
    assert any('torch built for cuda=13.0' in w for w in rep.warnings)
    assert any(
        'torch.cuda.is_available()=False despite a healthy driver' in w
        for w in rep.warnings
    )


def test_no_nvidia_smi_assumes_cpu_host():
    """CPU-only laptop / CI runner — should not error, just note it."""
    rep = check_host_environment(
        torch_mod=_fake_torch(version='2.6.0+cpu', cuda=None, available=False),
        nvidia_smi_output='',  # parse fails → "could not parse" warning
    )
    # parse failure produces a warning, not an error
    assert rep.ok is True
    assert any('parse' in w or 'CPU-only' in w for w in rep.warnings)


def test_torch_import_failure_is_an_error():
    """Helpers consume torch_mod=None to trigger the real import. We can't
    easily simulate the ImportError here without monkey-patching sys, so
    instead assert the contract: if _probe_torch fails (we approximate
    that by passing a mod whose .version.cuda access raises), the
    failure should land in errors[]."""
    class Bad:
        @property
        def __version__(self):  # noqa: D401, A003
            raise RuntimeError('synthetic torch failure')

        version = SimpleNamespace(cuda=None)
        cuda = SimpleNamespace(is_available=lambda: False)

    rep = check_host_environment(
        torch_mod=Bad(),
        nvidia_smi_output=_nvsmi('560.94', '12.6'),
    )
    assert rep.ok is False
    assert rep.errors


def test_render_contains_status_line():
    rep = check_host_environment(
        torch_mod=_fake_torch(version='2.6.0', cuda='12.6', available=True),
        nvidia_smi_output=_nvsmi('560.94', '12.6'),
    )
    text = rep.render()
    assert 'preflight:' in text
    assert 'driver=560.94' in text
    assert 'torch=2.6.0' in text
    assert 'status=OK' in text
