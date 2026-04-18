"""CPU-only mock segmentor — produces an elliptical mask per box.

Used for CI / unit tests / running the pipeline on machines without a
SAM2-capable GPU. It is not a replacement for the real backend; it only
keeps shapes + dtypes consistent so downstream consumers can be
exercised.
"""

from __future__ import annotations

from typing import TYPE_CHECKING

import numpy as np

from .base_backend import BaseBackend, SegmentResult

if TYPE_CHECKING:
    from rclpy.node import Node


class MockBackend(BaseBackend):
    """Box → filled-ellipse binary mask. No GPU, no model."""

    def __init__(self, node: Node) -> None:
        super().__init__(node)

    def load(self) -> None:
        self._loaded = True
        self._node.get_logger().info('MockBackend loaded (no model)')

    def unload(self) -> None:
        self._loaded = False

    def segment(
        self, image_rgb: np.ndarray, boxes_xyxy: np.ndarray,
    ) -> list[SegmentResult]:
        h, w = image_rgb.shape[:2]
        out: list[SegmentResult] = []
        for box in np.asarray(boxes_xyxy, dtype=np.float64):
            x1, y1, x2, y2 = box
            x1 = int(np.clip(x1, 0, w - 1))
            y1 = int(np.clip(y1, 0, h - 1))
            x2 = int(np.clip(x2, 0, w))
            y2 = int(np.clip(y2, 0, h))
            mask = np.zeros((h, w), dtype=bool)
            if x2 > x1 and y2 > y1:
                cy = 0.5 * (y1 + y2)
                cx = 0.5 * (x1 + x2)
                ry = max(0.5 * (y2 - y1), 1.0)
                rx = max(0.5 * (x2 - x1), 1.0)
                ys = np.arange(y1, y2)[:, None]
                xs = np.arange(x1, x2)[None, :]
                ellipse = ((ys - cy) / ry) ** 2 + ((xs - cx) / rx) ** 2 <= 1.0
                mask[y1:y2, x1:x2] = ellipse
            out.append(SegmentResult(mask=mask, score=1.0))
        return out
