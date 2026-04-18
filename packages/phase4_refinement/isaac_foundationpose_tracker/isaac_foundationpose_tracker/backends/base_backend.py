"""Abstract base for RGB-D 6D-pose estimation backends."""

from __future__ import annotations

from abc import ABC, abstractmethod
from dataclasses import dataclass
from typing import TYPE_CHECKING

import numpy as np

if TYPE_CHECKING:
    from rclpy.node import Node


@dataclass
class PoseResult:
    """Single per-detection 6D pose result.

    Attributes
    ----------
    pose_4x4 : (4, 4) float64 ndarray
        Rigid transform from object frame to camera frame.
    score : float
        Backend-reported confidence in [0, 1].
    class_name : str
        Mesh / object class the pose was estimated against.
    object_id : int
        ID of the source detection (copied through so the node does not
        need to re-pair with detections downstream).
    """

    pose_4x4: np.ndarray
    score: float
    class_name: str
    object_id: int


class BasePoseBackend(ABC):
    """6D pose estimation backend interface.

    The node hands over the current RGB image, aligned depth, camera
    intrinsics (K, 3x3), and per-detection prompts (box + class + id).
    The backend returns one :class:`PoseResult` per prompt it can solve
    for. Backends may drop prompts whose class has no registered mesh;
    the node preserves ordering but does not require one result per
    prompt.

    All heavy imports (torch, foundationpose, kaolin, …) MUST happen
    inside :meth:`load` so this module is import-safe on CPU-only hosts.
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
        depth_m: np.ndarray,
        K: np.ndarray,
        prompts: list[PosePrompt],
    ) -> list[PoseResult]:
        """Run 6D pose estimation.

        Parameters
        ----------
        image_rgb : (H, W, 3) uint8
            RGB image.
        depth_m : (H, W) float32
            Depth in metres, 0.0 for invalid pixels. Must be registered
            to ``image_rgb`` (same intrinsics, same size).
        K : (3, 3) float64
            Pinhole intrinsics.
        prompts : list[PosePrompt]
            Per-detection prompts. Backends may skip prompts without a
            registered mesh.

        Returns
        -------
        list[PoseResult]
            One result per solved prompt (subset of ``prompts``, same
            order).
        """


@dataclass
class PosePrompt:
    """Per-detection input to a pose backend."""

    object_id: int
    class_name: str
    bbox_xyxy: tuple[float, float, float, float]
    confidence: float
