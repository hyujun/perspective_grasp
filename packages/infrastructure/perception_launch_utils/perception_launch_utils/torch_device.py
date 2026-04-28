# Copyright 2026 perspective_grasp
# SPDX-License-Identifier: Apache-2.0
"""Torch device resolution shared by every GPU-using node in the workspace.

Why this exists
---------------
The host venv and a Phase 4 container can disagree about which CUDA build
of torch is installed (e.g. ``cu130`` wheel on a CUDA-12.8 driver host).
``torch.cuda.is_available()`` alone is not enough — on a driver/runtime
mismatch it can still raise on the first real allocation. Calling code
should ask this helper "give me a device string I can hand to
ultralytics / SAM2 / FoundationPose" and it returns one that has been
verified by an actual tensor allocation, falling back to ``"cpu"`` with
a single WARN log when GPU isn't usable.

Identifiers (``"auto"``, ``"cuda"``, ``"cuda:0"``, ``"0"``, ``"cpu"``)
follow the conventions used by ultralytics and torch.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Any, Optional, Protocol


class _Logger(Protocol):
    def info(self, msg: str) -> None: ...
    def warn(self, msg: str) -> None: ...
    def error(self, msg: str) -> None: ...


@dataclass(frozen=True)
class DeviceResolution:
    """Outcome of :func:`resolve_torch_device`.

    Attributes
    ----------
    device
        Full torch device string, e.g. ``"cuda:0"`` or ``"cpu"``. Pass
        this to ``model.to(device=...)``, ``ultralytics``, ``build_sam2``,
        etc.
    device_type
        Just the family — ``"cuda"`` or ``"cpu"`` — without the index.
        Required by APIs like ``torch.autocast(device_type=...)`` that
        reject ``"cuda:0"``.
    downgraded
        True iff the request was for a CUDA device but we had to fall
        back to CPU. Callers can use this to skip ``torch.autocast`` /
        ``half()`` paths that aren't worthwhile on CPU.
    reason
        Short human-readable explanation; empty when the request was
        satisfied as-is.
    """

    device: str
    device_type: str
    downgraded: bool
    reason: str
    driver_cuda: Optional[str] = None
    torch_cuda: Optional[str] = None


def _normalize(requested: str) -> str:
    """Map user-facing aliases to a canonical torch device string.

    ``"auto"`` is left as-is so the caller can decide later. Numeric
    ``"0"`` / ``"1"`` map to ``"cuda:N"``. Bare ``"cuda"`` collapses to
    ``"cuda:0"`` so logs and ``DeviceResolution.device`` are consistent
    regardless of how the YAML was written. Anything else (``"cpu"``,
    ``"mps"``, ``"cuda:1"``, ...) is returned untouched.
    """
    s = (requested or '').strip().lower()
    if not s:
        return 'auto'
    if s == 'cuda':
        return 'cuda:0'
    if s.isdigit():
        return f'cuda:{s}'
    return s


def _probe_cuda(torch_mod: Any, device: str) -> tuple[bool, str]:
    """Allocate a tiny tensor on ``device`` to confirm CUDA actually works.

    ``torch.cuda.is_available()`` returns True on some driver/runtime
    mismatches that then fail at first allocation (e.g. cu130 wheel on a
    CUDA-12.8 driver). We do the alloc here so the failure surfaces in
    one well-known place instead of inside a model's forward pass.
    """
    try:
        t = torch_mod.zeros(1, device=device)
        # Force a synchronous op so a deferred CUDA error materializes here.
        _ = t + 1
        del t
        if hasattr(torch_mod.cuda, 'synchronize'):
            torch_mod.cuda.synchronize()
        return True, ''
    except Exception as exc:  # noqa: BLE001 — we want every failure mode
        return False, f'{type(exc).__name__}: {exc}'


def resolve_torch_device(
    requested: str,
    logger: _Logger,
    *,
    torch_mod: Any = None,
) -> DeviceResolution:
    """Resolve a node's ``device`` parameter to a usable torch device.

    Parameters
    ----------
    requested
        The raw value of the node's ``device`` parameter. Accepts
        ``"auto"`` (pick the best available), ``"cuda"`` / ``"cuda:N"``
        / numeric ``"N"`` (CUDA), or ``"cpu"``.
    logger
        Anything with ``info`` / ``warn`` / ``error`` methods — typically
        ``node.get_logger()``. Strings are not formatted; pass plain
        ``str``.
    torch_mod
        Test seam. Production callers pass nothing; we ``import torch``
        lazily so this module remains importable on CPU-only / no-torch
        hosts (matches existing Phase 4 backend pattern).

    Returns
    -------
    :class:`DeviceResolution`
        Use ``.device`` as the value to hand to ultralytics / SAM2 /
        torch. Inspect ``.downgraded`` if the caller wants to disable
        GPU-specific code paths (e.g. ``torch.autocast``).
    """
    canonical = _normalize(requested)

    if torch_mod is None:
        try:
            import torch as torch_mod  # type: ignore
        except ImportError as exc:
            msg = f"torch not importable ({exc}); using cpu"
            logger.warn(msg)
            return DeviceResolution(
                device='cpu', device_type='cpu',
                downgraded=canonical != 'cpu', reason=msg,
            )

    torch_cuda = getattr(torch_mod.version, 'cuda', None)

    if canonical == 'cpu':
        return DeviceResolution(
            device='cpu', device_type='cpu', downgraded=False, reason='',
        )

    cuda_available = bool(getattr(torch_mod.cuda, 'is_available', lambda: False)())
    if not cuda_available:
        msg = (
            f"CUDA unavailable (torch.cuda.is_available()=False, "
            f"torch.version.cuda={torch_cuda}); falling back to cpu"
        )
        logger.warn(msg)
        return DeviceResolution(
            device='cpu', device_type='cpu',
            downgraded=True, reason=msg, torch_cuda=torch_cuda,
        )

    target = 'cuda:0' if canonical == 'auto' else canonical
    ok, err = _probe_cuda(torch_mod, target)
    if not ok:
        msg = (
            f"CUDA probe failed on {target} ({err}); falling back to cpu. "
            f"Likely driver/torch CUDA mismatch — torch built for "
            f"cuda={torch_cuda}, check `nvidia-smi` Driver/CUDA Version."
        )
        logger.warn(msg)
        return DeviceResolution(
            device='cpu', device_type='cpu',
            downgraded=True, reason=msg, torch_cuda=torch_cuda,
        )

    name = ''
    try:
        idx = int(target.split(':', 1)[1]) if ':' in target else 0
        name = torch_mod.cuda.get_device_name(idx)
    except Exception:  # noqa: BLE001
        pass
    logger.info(
        f"torch device={target} ({name}) torch.cuda={torch_cuda}"
        if name else f"torch device={target} torch.cuda={torch_cuda}"
    )
    device_type = target.split(':', 1)[0]
    return DeviceResolution(
        device=target, device_type=device_type,
        downgraded=False, reason='', torch_cuda=torch_cuda,
    )
