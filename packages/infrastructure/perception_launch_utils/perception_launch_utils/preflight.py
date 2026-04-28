# Copyright 2026 perspective_grasp
# SPDX-License-Identifier: Apache-2.0
"""Pre-launch host environment check.

The runtime helper :mod:`perception_launch_utils.torch_device` already
catches torch/driver mismatches at node startup and falls back to CPU.
This module is the *complementary* check: it runs once at launch time,
**before** any node spins, and prints a single block summarising the
host's GPU/torch state. The aim is purely diagnostic — when something
goes wrong, the user gets one well-formatted set of facts at the top
of the log instead of having to grep across N node stderrs.

The ``OpaqueFunction`` entry point (:func:`preflight_launch_action`)
returns ``[LogInfo(...)]`` so it composes cleanly into a normal
``LaunchDescription``. ``strict=True`` upgrades a detected mismatch
from a warning to a hard failure (``Shutdown``) — useful for CI runs.
"""

from __future__ import annotations

import os
import re
import shutil
import subprocess
from dataclasses import dataclass, field
from typing import Any, Optional


@dataclass(frozen=True)
class PreflightReport:
    """Result of the host environment probe.

    ``ok`` is True iff every probe succeeded *and* no version mismatch
    was detected. ``warnings`` collects every non-fatal finding;
    ``errors`` collects findings that flip ``ok`` to False under
    ``strict=True`` semantics.
    """

    ok: bool
    skipped: bool = False
    driver_version: Optional[str] = None
    driver_cuda: Optional[str] = None  # e.g. '12.8' from nvidia-smi
    torch_version: Optional[str] = None
    torch_cuda: Optional[str] = None  # e.g. '12.6' from torch.version.cuda
    cuda_available: Optional[bool] = None
    gpu_name: Optional[str] = None
    warnings: list[str] = field(default_factory=list)
    errors: list[str] = field(default_factory=list)

    def render(self) -> str:
        """Human-readable single block for log output."""
        if self.skipped:
            return 'preflight: skipped (PERSPECTIVE_PREFLIGHT_SKIP=1)'
        lines = ['preflight:']
        lines.append(
            f'  driver={self.driver_version or "?"}  '
            f'driver_cuda={self.driver_cuda or "?"}  '
            f'gpu={self.gpu_name or "?"}'
        )
        lines.append(
            f'  torch={self.torch_version or "?"}  '
            f'torch_cuda={self.torch_cuda or "?"}  '
            f'cuda_available={self.cuda_available}'
        )
        for w in self.warnings:
            lines.append(f'  WARN: {w}')
        for e in self.errors:
            lines.append(f'  ERROR: {e}')
        if self.ok and not self.warnings:
            lines.append('  status=OK')
        elif self.ok:
            lines.append('  status=OK (with warnings)')
        else:
            lines.append('  status=FAIL')
        return '\n'.join(lines)


def _parse_nvidia_smi(output: str) -> tuple[Optional[str], Optional[str], Optional[str]]:
    """Pull (driver_version, cuda_version, gpu_name) from ``nvidia-smi`` text.

    The default ``nvidia-smi`` output (no flags) prints a header table
    that contains both ``Driver Version: NNN.NNN.NN`` and
    ``CUDA Version: NN.N`` on the same line, plus a per-GPU row whose
    second column is the human-readable model name. We intentionally
    do not call ``nvidia-smi --query-gpu=...`` — that variant is
    fragile across very old drivers and we already have to parse the
    header for the CUDA Version field anyway.
    """
    driver = None
    cuda = None
    gpu = None

    m = re.search(r'Driver Version:\s*([0-9.]+)', output)
    if m:
        driver = m.group(1)
    m = re.search(r'CUDA Version:\s*([0-9.]+)', output)
    if m:
        cuda = m.group(1)
    # GPU model: first row in the per-GPU section. The format is fairly
    # stable across driver versions:
    #   |   0  NVIDIA GeForce RTX 4090 ...
    # We accept anything that looks like an NVIDIA model name.
    m = re.search(r'\b(NVIDIA\s+[\w\- ]+?)(?:\s{2,}|\s+On|\s+Off|\|)', output)
    if m:
        gpu = m.group(1).strip()
    return driver, cuda, gpu


def _probe_nvidia_smi() -> tuple[Optional[str], Optional[str], Optional[str], Optional[str]]:
    """Returns (driver, cuda, gpu, error_msg).

    ``error_msg`` is None on success. If ``nvidia-smi`` isn't on PATH,
    returns ``(None, None, None, 'nvidia-smi not on PATH')`` — caller
    decides whether that's a warning (CPU-only host) or an error.
    """
    if not shutil.which('nvidia-smi'):
        return None, None, None, 'nvidia-smi not on PATH'
    try:
        out = subprocess.check_output(
            ['nvidia-smi'], text=True, timeout=4,
            stderr=subprocess.STDOUT,
        )
    except Exception as exc:  # noqa: BLE001
        return None, None, None, f'nvidia-smi failed: {exc}'
    drv, cuda, gpu = _parse_nvidia_smi(out)
    if drv is None and cuda is None:
        return None, None, None, 'could not parse nvidia-smi output'
    return drv, cuda, gpu, None


def _probe_torch(
    torch_mod: Any = None,
    *,
    use_subprocess: bool = True,
) -> tuple[Optional[str], Optional[str], Optional[bool], Optional[str]]:
    """Returns (version, version.cuda, is_available, error_msg).

    The launch process is ``/opt/ros/jazzy/bin/ros2``, whose shebang is
    hardcoded ``/usr/bin/python3`` and therefore does not see packages
    installed into the workspace venv. Node subprocesses (started via
    ``#!/usr/bin/env python3``) DO see the venv because ``activate``
    prepends ``.venv/bin`` to PATH. So an in-process ``import torch``
    here would say "missing" even when the actual nodes will find it.

    To probe what nodes will actually see, we shell out to ``python3``
    via PATH. That picks up the same interpreter the nodes will run
    under. ``use_subprocess=False`` forces in-process import (used by
    the test seam where ``torch_mod`` is injected).
    """
    if torch_mod is not None:
        try:
            ver = getattr(torch_mod, '__version__', None)
            cuda = getattr(torch_mod.version, 'cuda', None)
            avail = bool(getattr(torch_mod.cuda, 'is_available', lambda: False)())
            return ver, cuda, avail, None
        except Exception as exc:  # noqa: BLE001
            return None, None, None, f'torch probe failed: {exc}'

    if not use_subprocess:
        try:
            import torch as _t  # type: ignore
        except ImportError as exc:
            return None, None, None, f'torch not importable: {exc}'
        return _probe_torch(torch_mod=_t, use_subprocess=False)

    # Subprocess path: emit JSON on stdout so we don't have to parse
    # torch's own startup chatter (deprecation warnings etc. on stderr).
    py_code = (
        'import json,sys\n'
        'try:\n'
        '    import torch\n'
        '    print(json.dumps({"v":torch.__version__,"c":torch.version.cuda,'
        '"a":bool(torch.cuda.is_available())}))\n'
        'except Exception as e:\n'
        '    print(json.dumps({"err":f"{type(e).__name__}: {e}"}))\n'
    )
    try:
        out = subprocess.check_output(
            ['python3', '-c', py_code],
            text=True, timeout=10,
            stderr=subprocess.PIPE,
        )
    except FileNotFoundError:
        return None, None, None, 'python3 not on PATH'
    except subprocess.TimeoutExpired:
        return None, None, None, 'torch import timed out'
    except subprocess.CalledProcessError as exc:
        return None, None, None, f'subprocess error: {exc.stderr or exc}'
    import json
    line = out.strip().splitlines()[-1] if out.strip() else '{}'
    try:
        d = json.loads(line)
    except json.JSONDecodeError:
        return None, None, None, f'unparseable torch probe output: {line!r}'
    if 'err' in d:
        return None, None, None, f'torch not importable: {d["err"]}'
    return d.get('v'), d.get('c'), bool(d.get('a')), None


def _cuda_minor(version: Optional[str]) -> Optional[tuple[int, int]]:
    """``'12.8'`` → ``(12, 8)``; falsy / unparseable → ``None``."""
    if not version:
        return None
    parts = version.split('.')
    try:
        major = int(parts[0])
        minor = int(parts[1]) if len(parts) > 1 else 0
        return major, minor
    except (ValueError, IndexError):
        return None


def check_host_environment(
    *,
    skip: bool = False,
    torch_mod: Any = None,
    nvidia_smi_output: Optional[str] = None,
) -> PreflightReport:
    """Probe the host and return a :class:`PreflightReport`.

    Parameters
    ----------
    skip
        Bypass everything and return ``PreflightReport(ok=True,
        skipped=True)``. Wire-able to a launch arg / env var so CI
        tests don't have to mock nvidia-smi.
    torch_mod, nvidia_smi_output
        Test seams. Production callers leave these as None.
    """
    if skip:
        return PreflightReport(ok=True, skipped=True)

    warnings: list[str] = []
    errors: list[str] = []

    if nvidia_smi_output is not None:
        drv, drv_cuda, gpu = _parse_nvidia_smi(nvidia_smi_output)
        # An empty / unparseable string from the test seam should produce
        # the same warning as a missing nvidia-smi at runtime.
        nvsmi_err: Optional[str] = (
            None if (drv or drv_cuda)
            else 'nvidia-smi output empty or unparseable'
        )
    else:
        drv, drv_cuda, gpu, nvsmi_err = _probe_nvidia_smi()
    if nvsmi_err:
        warnings.append(nvsmi_err + ' — assuming CPU-only host')

    tver, tcuda, tavail, terr = _probe_torch(torch_mod=torch_mod)
    if terr:
        errors.append(terr)

    # The actual mismatch check — only meaningful when we have both sides.
    drv_t = _cuda_minor(drv_cuda)
    t_t = _cuda_minor(tcuda)
    if drv_t and t_t and t_t > drv_t:
        warnings.append(
            f'torch built for cuda={tcuda} but driver supports up to '
            f'cuda={drv_cuda} — runtime will fall back to CPU. Reinstall '
            f'torch with PERSPECTIVE_TORCH_CUDA matching the driver '
            f'(see docs/installation.md).'
        )
    if tavail is False and drv is not None:
        warnings.append(
            'torch.cuda.is_available()=False despite a healthy driver. '
            'Most common cause: torch CUDA build does not match driver. '
            'See CLAUDE.md anti-pattern (p).'
        )

    ok = not errors  # warnings alone do not flip ok=False
    return PreflightReport(
        ok=ok,
        driver_version=drv,
        driver_cuda=drv_cuda,
        torch_version=tver,
        torch_cuda=tcuda,
        cuda_available=tavail,
        gpu_name=gpu,
        warnings=warnings,
        errors=errors,
    )


# ---------------------------------------------------------------------------
# Launch integration
# ---------------------------------------------------------------------------

_SKIP_ENV = 'PERSPECTIVE_PREFLIGHT_SKIP'


def preflight_launch_action(*, strict: Any = False):
    """``OpaqueFunction``-compatible callable that runs the probe.

    Returns ``[LogInfo(...)]`` so the report appears at the top of the
    launch log. ``strict`` accepts either a literal ``bool`` or a
    ``LaunchConfiguration`` (read at evaluation time). When truthy and
    the report is not OK, a ``Shutdown`` action is appended so launch
    aborts before nodes start. When the report is OK but has warnings,
    strict mode does *not* abort — strict only fires on hard errors.

    Usage in a launch file::

        from launch.actions import OpaqueFunction
        from launch.substitutions import LaunchConfiguration
        from perception_launch_utils import preflight_launch_action

        return LaunchDescription([
            OpaqueFunction(
                function=preflight_launch_action(
                    strict=LaunchConfiguration('preflight_strict'),
                ),
            ),
            ...other actions...
        ])
    """
    # Imported lazily so this module remains importable in pure-pytest
    # contexts (the test suite has no `launch` package available when
    # ROS isn't sourced).
    from launch.actions import LogInfo, Shutdown

    def _resolve_strict(context) -> bool:
        if isinstance(strict, bool):
            return strict
        # Anything with a .perform(context) — LaunchConfiguration,
        # PerformSubstitution, etc. Falls through to str() otherwise.
        if hasattr(strict, 'perform'):
            value = strict.perform(context)
        else:
            value = str(strict)
        return value.strip().lower() in ('1', 'true', 'yes', 'on')

    def _impl(context, *_a, **_kw):
        skip = os.environ.get(_SKIP_ENV, '').lower() in ('1', 'true', 'yes')
        report = check_host_environment(skip=skip)
        actions = [LogInfo(msg=report.render())]
        if _resolve_strict(context) and not report.ok:
            actions.append(Shutdown(reason='preflight failed (strict mode)'))
        return actions

    return _impl
