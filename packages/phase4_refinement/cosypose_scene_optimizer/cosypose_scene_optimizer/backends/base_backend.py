"""Abstract base for scene-level (multi-object) pose optimization backends."""

from __future__ import annotations

from abc import ABC, abstractmethod
from dataclasses import dataclass
from typing import TYPE_CHECKING

import numpy as np

if TYPE_CHECKING:
    from rclpy.node import Node


@dataclass
class SceneDetection:
    """Per-detection input to the scene optimizer.

    Mirrors ``PosePrompt`` from the FoundationPose backend contract so
    the two packages stay shaped alike — the shared vocabulary is worth
    a tiny duplication.
    """

    object_id: int
    class_name: str
    bbox_xyxy: tuple[float, float, float, float]
    confidence: float


@dataclass
class SceneResult:
    """Single optimized 6D pose from a scene solve.

    Attributes
    ----------
    pose_4x4 : (4, 4) float64 ndarray
        Rigid transform from object frame to camera frame.
    score : float
        Backend-reported confidence in [0, 1].
    class_name : str
        Mesh / object class the pose is for.
    object_id : int
        ID of the source detection (carried through).
    """

    pose_4x4: np.ndarray
    score: float
    class_name: str
    object_id: int


class BaseSceneBackend(ABC):
    """Scene-level pose estimation interface.

    Solves all visible detections in a single image *jointly* so that
    occlusion / mutual-support constraints are respected. Compared with
    the FoundationPose per-detection backend, this returns at most one
    result per detection but may drop detections whose class has no
    registered mesh / whose score falls below the backend threshold.

    Heavy imports (torch, happypose, pytorch3d, …) MUST happen inside
    :meth:`load` so the module is import-safe on CPU-only hosts.
    """

    def __init__(self, node: Node) -> None:
        self._node = node
        self._loaded = False

    @property
    def loaded(self) -> bool:
        return self._loaded

    @abstractmethod
    def load(self) -> None:
        """Load weights / move to device. Called from on_activate."""

    @abstractmethod
    def unload(self) -> None:
        """Free GPU memory / drop references. Called from on_deactivate."""

    @abstractmethod
    def optimize(
        self,
        image_rgb: np.ndarray,
        K: np.ndarray,
        detections: list[SceneDetection],
        target_classes: list[str] | None = None,
    ) -> list[SceneResult]:
        """Run a scene-level solve.

        Parameters
        ----------
        image_rgb : (H, W, 3) uint8
            RGB image.
        K : (3, 3) float64
            Pinhole intrinsics.
        detections : list[SceneDetection]
            Per-object prompts. Backends may drop prompts with no mesh
            registered.
        target_classes : list[str] | None
            Optional filter — when non-empty, prompts whose class name
            is not in the list are skipped before inference.

        Returns
        -------
        list[SceneResult]
            One result per solved detection (subset, same order).
        """
