"""Abstract base for box-prompted instance segmentation backends."""

from __future__ import annotations

from abc import ABC, abstractmethod
from dataclasses import dataclass
from typing import TYPE_CHECKING

import numpy as np

if TYPE_CHECKING:
    from rclpy.node import Node


@dataclass
class SegmentResult:
    """Single per-box segmentation result.

    Attributes
    ----------
    mask : (H, W) bool ndarray
        True where the object is present.
    score : float
        Backend-reported confidence / IoU prediction in [0, 1].
    """
    mask: np.ndarray
    score: float


class BaseBackend(ABC):
    """Segmentation backend interface.

    The node hands over the current RGB image and a list of pixel-space
    bounding boxes. The backend returns one :class:`SegmentResult` per
    box. All heavy imports (torch, sam2, …) must happen inside
    :meth:`load` so the module itself is import-safe on CPU-only hosts.
    """

    def __init__(self, node: Node) -> None:
        self._node = node
        self._loaded = False

    @property
    def loaded(self) -> bool:
        return self._loaded

    @abstractmethod
    def load(self) -> None:
        """Load weights / move to device. Called from lifecycle on_activate."""

    @abstractmethod
    def unload(self) -> None:
        """Free GPU memory / drop references. Called from on_deactivate."""

    @abstractmethod
    def segment(
        self, image_rgb: np.ndarray, boxes_xyxy: np.ndarray,
    ) -> list[SegmentResult]:
        """Run segmentation.

        Parameters
        ----------
        image_rgb : (H, W, 3) uint8
            RGB image.
        boxes_xyxy : (N, 4) float
            Pixel-space bounding boxes [x1, y1, x2, y2].

        Returns
        -------
        list[SegmentResult]
            Length N. ``i``-th entry is the mask for box ``i``.
        """
