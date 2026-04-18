"""Abstract base for BundleSDF-style unknown-object trackers.

BundleSDF differs from the pose-per-frame backends (FoundationPose /
MegaPose / CosyPose) in that it builds a neural SDF + pose graph over
a sequence of frames for each tracked object. That means:

* Persistent per-track state. A single ``track_id`` must be fed
  successive frames; the backend decides when to initialise, update,
  or drop internal state.
* Mask-driven, not box-driven. BundleSDF segments its target from
  depth using SAM2's binary mask, so the prompt carries an (H, W)
  bool mask rather than an xyxy bbox.
* Unknown-class safe. ``class_name`` is optional — SAM2's output does
  not carry one, and BundleSDF does not require a mesh prior.

Keep this module light — no torch / open3d / BundleSDF imports.
"""

from __future__ import annotations

from abc import ABC, abstractmethod
from dataclasses import dataclass
from typing import TYPE_CHECKING

import numpy as np

if TYPE_CHECKING:
    from rclpy.node import Node


@dataclass
class TrackPrompt:
    """Per-mask input to a BundleSDF-style tracker.

    ``mask`` is a boolean (H, W) array aligned with the RGB / depth
    image. ``track_id`` identifies the object across frames — the
    backend keys its internal SDF / pose state on it so calling
    :meth:`BaseTrackerBackend.track` with the same id across frames
    incrementally improves the estimate.
    """

    track_id: int
    mask: np.ndarray
    class_name: str = ''
    confidence: float = 0.0


@dataclass
class TrackResult:
    """Single per-track 6D pose result.

    Attributes
    ----------
    pose_4x4 : (4, 4) float64 ndarray
        Rigid transform from object frame to camera frame.
    score : float
        Backend-reported confidence in [0, 1].
    track_id : int
        ID of the source track (passed through from the prompt).
    class_name : str
        Echoed from the prompt (may be empty).
    """

    pose_4x4: np.ndarray
    score: float
    track_id: int
    class_name: str = ''


class BaseTrackerBackend(ABC):
    """Unknown-object 6D tracker interface.

    The node hands over the current RGB image, aligned depth, camera
    intrinsics (K, 3x3), and per-object prompts (mask + id). The backend
    returns one :class:`TrackResult` per prompt it can produce a pose
    for.

    Contract details:

    * ``track()`` is called once per RGB-D frame. Backends are free to
      skip a prompt (e.g. mask too small, depth invalid) without
      returning a result for it.
    * ``track_id`` is persistent across frames. Backends should keep
      per-id state; the node does not re-initialise between calls.
    * All heavy imports (torch, open3d, pytorch3d, BundleSDF native
      ops) MUST happen inside :meth:`load` so this module is
      import-safe on CPU-only hosts.
    """

    def __init__(self, node: Node) -> None:
        self._node = node
        self._loaded = False

    @property
    def loaded(self) -> bool:
        return self._loaded

    @abstractmethod
    def load(self) -> None:
        """Initialise weights / configs / device. Called from on_activate."""

    @abstractmethod
    def unload(self) -> None:
        """Free GPU memory / drop references. Called from on_deactivate."""

    @abstractmethod
    def track(
        self,
        image_rgb: np.ndarray,
        depth_m: np.ndarray,
        K: np.ndarray,
        prompts: list[TrackPrompt],
    ) -> list[TrackResult]:
        """Run one step of BundleSDF-style tracking.

        Parameters
        ----------
        image_rgb : (H, W, 3) uint8
            RGB image.
        depth_m : (H, W) float32
            Depth in metres, 0.0 for invalid pixels.
        K : (3, 3) float64
            Pinhole intrinsics.
        prompts : list[TrackPrompt]
            One entry per object to track this frame.

        Returns
        -------
        list[TrackResult]
            One result per prompt the backend could solve for (subset
            of ``prompts`` — order is not required to match).
        """

    def drop_track(self, track_id: int) -> None:  # noqa: D401
        """Drop per-track state for ``track_id``.

        Default no-op. Override to free SDF / pose-graph memory when
        the node notices a track has gone stale.
        """
