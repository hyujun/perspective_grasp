"""CPU-only mock tracker backend.

Back-projects the centroid of each mask through the aligned depth
image to produce a translation, picks an identity rotation, and
emits a ``TrackResult``. No SDF, no pose graph — just enough signal
to exercise the topic / lifecycle path without a BundleSDF-capable
GPU.
"""

from __future__ import annotations

from typing import TYPE_CHECKING

import numpy as np

from .base_backend import BaseTrackerBackend, TrackPrompt, TrackResult

if TYPE_CHECKING:
    from rclpy.node import Node


class MockTrackerBackend(BaseTrackerBackend):
    """Depth-unprojected mask-centroid pose. No GPU, no model."""

    def __init__(self, node: Node) -> None:
        super().__init__(node)

    def load(self) -> None:
        self._loaded = True
        self._node.get_logger().info('MockTrackerBackend loaded (no model)')

    def unload(self) -> None:
        self._loaded = False

    def track(
        self,
        image_rgb: np.ndarray,
        depth_m: np.ndarray,
        K: np.ndarray,
        prompts: list[TrackPrompt],
    ) -> list[TrackResult]:
        if depth_m.shape[:2] != image_rgb.shape[:2]:
            raise ValueError(
                f'depth {depth_m.shape[:2]} != rgb {image_rgb.shape[:2]}'
            )
        h, w = depth_m.shape
        fx = float(K[0, 0])
        fy = float(K[1, 1])
        cx = float(K[0, 2])
        cy = float(K[1, 2])

        out: list[TrackResult] = []
        for p in prompts:
            if p.mask.shape != (h, w):
                continue
            ys, xs = np.where(p.mask)
            if xs.size == 0:
                continue

            # Median of in-mask pixels with valid depth → robust to edge noise.
            u = int(np.median(xs))
            v = int(np.median(ys))
            z_patch = depth_m[p.mask & (depth_m > 0.0)]
            if z_patch.size == 0:
                continue
            z = float(np.median(z_patch))
            if not np.isfinite(z) or z <= 0.0:
                continue

            X = (u - cx) * z / fx
            Y = (v - cy) * z / fy
            T = np.eye(4, dtype=np.float64)
            T[:3, 3] = (X, Y, z)
            out.append(TrackResult(
                pose_4x4=T,
                score=float(p.confidence),
                track_id=p.track_id,
                class_name=p.class_name,
            ))
        return out
