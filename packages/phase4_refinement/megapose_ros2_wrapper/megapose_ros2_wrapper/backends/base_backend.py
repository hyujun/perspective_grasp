"""Abstract base for RGB-only zero-shot 6D pose estimation backends."""

from __future__ import annotations

from abc import ABC, abstractmethod
from dataclasses import dataclass
from typing import TYPE_CHECKING

import numpy as np

if TYPE_CHECKING:
    from rclpy.node import Node


@dataclass
class MegaPrompt:
    """Per-detection input to a zero-shot pose backend."""

    object_id: int
    class_name: str
    bbox_xyxy: tuple[float, float, float, float]
    confidence: float


@dataclass
class MegaResult:
    """Single per-detection zero-shot 6D pose result.

    Attributes
    ----------
    pose_4x4 : (4, 4) float64 ndarray
        Rigid transform from object frame to camera frame.
    score : float
        Backend-reported confidence in [0, 1].
    class_name : str
        Mesh / object class the pose is for (carried through).
    object_id : int
        ID of the source detection (carried through).
    """

    pose_4x4: np.ndarray
    score: float
    class_name: str
    object_id: int


class BaseMegaBackend(ABC):
    """Zero-shot RGB-only 6D pose estimation interface.

    The node hands over the current RGB image, camera intrinsics (K,
    3x3), and per-detection prompts (box + class + id). The backend
    returns one :class:`MegaResult` per prompt it can solve for.
    Depth is intentionally omitted — MegaPose's default mode is RGB+
    mesh. If a deployment adds depth later, extend this contract, do
    not silently pass it through.

    Heavy imports (torch, happypose, pytorch3d, …) MUST happen inside
    :meth:`load` so this module is import-safe on CPU-only hosts.
    """

    def __init__(self, node: Node) -> None:
        self._node = node
        self._loaded = False

    @property
    def loaded(self) -> bool:
        return self._loaded

    @abstractmethod
    def load(self) -> None:
        """Load weights + meshes / move to device. Called from on_activate."""

    @abstractmethod
    def unload(self) -> None:
        """Free GPU memory / drop references. Called from on_deactivate."""

    @abstractmethod
    def estimate(
        self,
        image_rgb: np.ndarray,
        K: np.ndarray,
        prompts: list[MegaPrompt],
    ) -> list[MegaResult]:
        """Run zero-shot 6D pose estimation.

        Parameters
        ----------
        image_rgb : (H, W, 3) uint8
            RGB image.
        K : (3, 3) float64
            Pinhole intrinsics.
        prompts : list[MegaPrompt]
            Per-detection prompts. Backends may drop prompts without
            a registered mesh.

        Returns
        -------
        list[MegaResult]
            One result per solved prompt (subset, same order).
        """
