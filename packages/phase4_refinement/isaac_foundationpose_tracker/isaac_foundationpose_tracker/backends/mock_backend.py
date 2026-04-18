"""CPU-only mock pose backend.

Back-projects each prompt's bbox centre through the aligned depth image
to produce a translation, picks an identity rotation, and emits a
``PoseResult``. Keeps shapes / dtypes / topics exercised so downstream
nodes (pose_filter, grasp_planner) can be smoke-tested without a
FoundationPose-capable GPU.
"""

from __future__ import annotations

from typing import TYPE_CHECKING

import numpy as np

from .base_backend import BasePoseBackend, PosePrompt, PoseResult

if TYPE_CHECKING:
    from rclpy.node import Node


class MockPoseBackend(BasePoseBackend):
    """Depth-unprojected bbox-centre pose. No GPU, no model."""

    def __init__(self, node: Node) -> None:
        super().__init__(node)

    def load(self) -> None:
        self._loaded = True
        self._node.get_logger().info('MockPoseBackend loaded (no model)')

    def unload(self) -> None:
        self._loaded = False

    def estimate(
        self,
        image_rgb: np.ndarray,
        depth_m: np.ndarray,
        K: np.ndarray,
        prompts: list[PosePrompt],
    ) -> list[PoseResult]:
        if depth_m.shape[:2] != image_rgb.shape[:2]:
            raise ValueError(
                f'depth {depth_m.shape[:2]} != rgb {image_rgb.shape[:2]}'
            )
        h, w = depth_m.shape
        fx = float(K[0, 0])
        fy = float(K[1, 1])
        cx = float(K[0, 2])
        cy = float(K[1, 2])

        out: list[PoseResult] = []
        for p in prompts:
            x1, y1, x2, y2 = p.bbox_xyxy
            u = int(np.clip(0.5 * (x1 + x2), 0, w - 1))
            v = int(np.clip(0.5 * (y1 + y2), 0, h - 1))
            z = float(depth_m[v, u])
            if not np.isfinite(z) or z <= 0.0:
                continue

            X = (u - cx) * z / fx
            Y = (v - cy) * z / fy
            T = np.eye(4, dtype=np.float64)
            T[:3, 3] = (X, Y, z)
            out.append(PoseResult(
                pose_4x4=T,
                score=float(p.confidence),
                class_name=p.class_name,
                object_id=p.object_id,
            ))
        return out
