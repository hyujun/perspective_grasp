"""CPU-only mock pose backend.

Places each prompt at a fixed depth (param ``mock_default_depth_m``,
default 1 m) back-projected from the bbox centre through the camera
intrinsics. Pure heuristic — keeps shapes / dtypes / topics exercised
so downstream nodes (pose_filter, grasp_planner) can be smoke-tested
without happypose weights.
"""

from __future__ import annotations

from typing import TYPE_CHECKING

import numpy as np

from .base_backend import BaseMegaBackend, MegaPrompt, MegaResult

if TYPE_CHECKING:
    from rclpy.node import Node


class MockMegaBackend(BaseMegaBackend):
    """Bbox-centre back-projection at fixed depth. No GPU, no model."""

    def __init__(self, node: Node) -> None:
        super().__init__(node)
        self._default_depth_m: float = (
            node.declare_parameter('mock_default_depth_m', 1.0)
            .get_parameter_value().double_value
        )

    def load(self) -> None:
        self._loaded = True
        self._node.get_logger().info(
            f'MockMegaBackend loaded (depth={self._default_depth_m}m)'
        )

    def unload(self) -> None:
        self._loaded = False

    def estimate(
        self,
        image_rgb: np.ndarray,
        K: np.ndarray,
        prompts: list[MegaPrompt],
    ) -> list[MegaResult]:
        h, w = image_rgb.shape[:2]
        fx = float(K[0, 0])
        fy = float(K[1, 1])
        cx = float(K[0, 2])
        cy = float(K[1, 2])
        z = self._default_depth_m

        out: list[MegaResult] = []
        for p in prompts:
            x1, y1, x2, y2 = p.bbox_xyxy
            u = float(np.clip(0.5 * (x1 + x2), 0.0, w - 1))
            v = float(np.clip(0.5 * (y1 + y2), 0.0, h - 1))
            X = (u - cx) * z / fx
            Y = (v - cy) * z / fy
            T = np.eye(4, dtype=np.float64)
            T[:3, 3] = (X, Y, z)
            out.append(MegaResult(
                pose_4x4=T,
                score=float(p.confidence),
                class_name=p.class_name,
                object_id=p.object_id,
            ))
        return out
