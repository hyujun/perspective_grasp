"""Meta AI SAM2 backend (``sam2.sam2_image_predictor.SAM2ImagePredictor``).

Runs inside the ``sam2`` Docker service; host installs are not supported
because of the PyTorch/CUDA coupling. All heavy imports are deferred to
:meth:`load` so the module can be imported for type-checking on
CPU-only hosts.
"""

from __future__ import annotations

from typing import TYPE_CHECKING, Any

import numpy as np

from .base_backend import BaseBackend, SegmentResult

if TYPE_CHECKING:
    from rclpy.node import Node


class Sam2Backend(BaseBackend):
    """Box-prompted SAM2 segmentation."""

    def __init__(self, node: Node) -> None:
        super().__init__(node)

        self._checkpoint: str = (
            node.declare_parameter('model_checkpoint', '')
            .get_parameter_value().string_value
        )
        self._model_cfg: str = (
            node.declare_parameter('model_config', 'sam2_hiera_l.yaml')
            .get_parameter_value().string_value
        )
        # Stored as-is here; the actual probe + fallback runs in load()
        # so that importing this module on CPU-only hosts (colcon test)
        # never reaches torch. 'auto' = cuda if usable else cpu.
        self._requested_device: str = (
            node.declare_parameter('device', 'auto')
            .get_parameter_value().string_value
        )
        self._device: str = ''  # populated by load() via resolve_torch_device
        # device_type ('cuda'/'cpu') is what torch.autocast accepts; 'cuda:0' would error.
        self._device_type: str = ''
        self._pred_iou_thresh: float = (
            node.declare_parameter('pred_iou_thresh', 0.88)
            .get_parameter_value().double_value
        )
        self._multimask: bool = (
            node.declare_parameter('multimask_output', False)
            .get_parameter_value().bool_value
        )

        self._predictor: Any = None  # SAM2ImagePredictor
        self._torch: Any = None

    # ------------------------------------------------------------------
    # Lifecycle
    # ------------------------------------------------------------------

    def load(self) -> None:
        if self._loaded:
            return
        if not self._checkpoint:
            raise RuntimeError(
                'model_checkpoint parameter is empty — cannot load SAM2'
            )

        import torch  # type: ignore
        from sam2.build_sam import build_sam2  # type: ignore
        from sam2.sam2_image_predictor import SAM2ImagePredictor  # type: ignore
        from perception_launch_utils import resolve_torch_device

        self._torch = torch
        resolution = resolve_torch_device(
            self._requested_device, self._node.get_logger(), torch_mod=torch,
        )
        self._device = resolution.device
        self._device_type = resolution.device_type
        self._node.get_logger().info(
            f'Loading SAM2 cfg={self._model_cfg} '
            f'ckpt={self._checkpoint} device={self._device}'
        )
        model = build_sam2(self._model_cfg, self._checkpoint, device=self._device)
        self._predictor = SAM2ImagePredictor(model)
        self._loaded = True
        self._node.get_logger().info('SAM2 loaded')

    def unload(self) -> None:
        self._predictor = None
        if self._torch is not None:
            try:
                self._torch.cuda.empty_cache()
            except Exception:  # noqa: BLE001
                pass
        self._loaded = False

    # ------------------------------------------------------------------
    # Inference
    # ------------------------------------------------------------------

    def segment(
        self, image_rgb: np.ndarray, boxes_xyxy: np.ndarray,
    ) -> list[SegmentResult]:
        if not self._loaded or self._predictor is None:
            raise RuntimeError('Sam2Backend.segment called before load()')

        boxes = np.asarray(boxes_xyxy, dtype=np.float32)
        if boxes.ndim != 2 or boxes.shape[1] != 4:
            raise ValueError(f'expected (N, 4) boxes, got shape {boxes.shape}')

        torch = self._torch
        with torch.inference_mode(), torch.autocast(self._device_type, dtype=torch.bfloat16):
            self._predictor.set_image(image_rgb)
            # SAM2ImagePredictor.predict accepts a batch of boxes via
            # ``box=np.array([[x1,y1,x2,y2], ...])`` → outputs per-box masks.
            masks, scores, _ = self._predictor.predict(
                box=boxes,
                multimask_output=self._multimask,
            )

        # Shape normalisation: ensure masks is (N, H, W).
        masks_np = np.asarray(masks)
        scores_np = np.asarray(scores)
        if masks_np.ndim == 2:  # single box case → (H, W)
            masks_np = masks_np[None]
            scores_np = np.atleast_1d(scores_np)
        elif masks_np.ndim == 4:  # multimask_output → (N, K, H, W)
            best = np.argmax(scores_np, axis=1)
            masks_np = masks_np[np.arange(masks_np.shape[0]), best]
            scores_np = scores_np[np.arange(scores_np.shape[0]), best]

        out: list[SegmentResult] = []
        for i in range(masks_np.shape[0]):
            out.append(SegmentResult(
                mask=masks_np[i].astype(bool),
                score=float(scores_np[i]) if scores_np.ndim else float(scores_np),
            ))
        return out
