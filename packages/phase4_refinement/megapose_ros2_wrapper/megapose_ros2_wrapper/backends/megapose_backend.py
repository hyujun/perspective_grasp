"""MegaPose zero-shot RGB 6D pose estimation via happypose.

Runs inside the shared ``cosypose-runtime`` Docker stage (same
happypose install). Heavy imports live in :meth:`load` so the module
is import-safe on CPU-only hosts.

Design notes
------------
* **Mesh registry from disk.** Unlike CosyPose (dataset-driven),
  MegaPose is zero-shot and can accept any mesh at inference — we
  scan ``mesh_dir`` for ``<class_name>.(obj|ply|stl)`` at load time
  and index one label → path entry per mesh. A prompt whose class
  has no mesh is silently dropped (logged at debug level).
* **Refiner-only integration.** Initial wiring runs happypose's
  default coarse + refine pipeline; no iterative self-supervision
  loop. Depth fusion can be added later if accuracy demands it.
* **API risk.** happypose is pre-1.0; the import paths and
  ``run_inference_pipeline`` signature below are the stable-looking
  surfaces as of 2026-04. First live run should log ``pip show
  happypose`` in the node so upstream drift surfaces loudly.
"""

from __future__ import annotations

import os
import subprocess
from typing import TYPE_CHECKING, Any

import numpy as np

from .base_backend import BaseMegaBackend, MegaPrompt, MegaResult

if TYPE_CHECKING:
    from rclpy.node import Node


_MESH_EXTS = ('.obj', '.ply', '.stl')


class MegaPoseBackend(BaseMegaBackend):
    """Zero-shot RGB 6D pose estimation via happypose/MegaPose."""

    def __init__(self, node: Node) -> None:
        super().__init__(node)

        self._dataset_dir: str = (
            node.declare_parameter('dataset_dir', '')
            .get_parameter_value().string_value
        )
        self._mesh_dir: str = (
            node.declare_parameter('mesh_dir', '')
            .get_parameter_value().string_value
        )
        self._model_name: str = (
            node.declare_parameter(
                'model_name', 'megapose-1.0-RGB-multi-hypothesis',
            ).get_parameter_value().string_value
        )
        self._n_refiner_iterations: int = (
            node.declare_parameter('n_refiner_iterations', 5)
            .get_parameter_value().integer_value
        )
        self._score_threshold: float = (
            node.declare_parameter('score_threshold', 0.3)
            .get_parameter_value().double_value
        )
        self._device: str = (
            node.declare_parameter('device', 'cuda')
            .get_parameter_value().string_value
        )

        self._torch: Any = None
        self._estimator: Any = None
        self._observation_cls: Any = None
        self._make_detections_fn: Any = None
        self._meshes: dict[str, str] = {}

    # ------------------------------------------------------------------
    # Lifecycle
    # ------------------------------------------------------------------

    def load(self) -> None:
        if self._loaded:
            return
        if not self._mesh_dir or not os.path.isdir(self._mesh_dir):
            raise RuntimeError(
                f"mesh_dir '{self._mesh_dir}' is missing or not a directory"
            )
        if self._dataset_dir:
            os.environ.setdefault('HAPPYPOSE_DATA_DIR', self._dataset_dir)

        _log_happypose_version(self._node)

        import torch  # type: ignore
        from happypose.toolbox.inference.types import (  # type: ignore
            ObservationTensor,
        )
        from happypose.toolbox.inference.utils import (  # type: ignore
            make_detections_from_object_data,
        )
        from happypose.pose_estimators.megapose.inference.pose_estimator import (  # type: ignore
            PoseEstimator,
        )
        from happypose.pose_estimators.megapose.inference.utils import (  # type: ignore
            load_named_model,
        )

        self._torch = torch
        self._observation_cls = ObservationTensor
        self._make_detections_fn = make_detections_from_object_data

        meshes = _discover_meshes(self._mesh_dir)
        if not meshes:
            raise RuntimeError(
                f"no meshes ({'|'.join(_MESH_EXTS)}) found in {self._mesh_dir}"
            )
        self._meshes = meshes

        self._node.get_logger().info(
            f'Loading MegaPose model={self._model_name} '
            f'meshes={len(meshes)} on {self._device}'
        )
        model = load_named_model(
            self._model_name,
            mesh_paths=meshes,
        )
        self._estimator = PoseEstimator(
            model=model,
            device=self._device,
        )

        self._loaded = True
        self._node.get_logger().info('MegaPose loaded')

    def unload(self) -> None:
        self._estimator = None
        self._meshes.clear()
        if self._torch is not None:
            try:
                self._torch.cuda.empty_cache()
            except Exception:  # noqa: BLE001
                pass
        self._loaded = False

    # ------------------------------------------------------------------
    # Inference
    # ------------------------------------------------------------------

    def estimate(
        self,
        image_rgb: np.ndarray,
        K: np.ndarray,
        prompts: list[MegaPrompt],
    ) -> list[MegaResult]:
        if not self._loaded or self._estimator is None:
            raise RuntimeError('MegaPoseBackend.estimate called before load()')

        kept: list[MegaPrompt] = []
        object_data: list[dict[str, Any]] = []
        for p in prompts:
            if p.class_name not in self._meshes:
                self._node.get_logger().debug(
                    f'no mesh registered for class {p.class_name!r}, skipping'
                )
                continue
            kept.append(p)
            object_data.append({
                'label': p.class_name,
                'bbox_modal': list(p.bbox_xyxy),
                'score': float(p.confidence),
            })
        if not kept:
            return []

        detections_tensor = self._make_detections_fn(object_data)
        obs = self._observation_cls.from_numpy(
            rgb=image_rgb, K=np.asarray(K, dtype=np.float64),
        )
        predictions, _extras = self._estimator.run_inference_pipeline(
            observation=obs.to(self._device),
            detections=detections_tensor.to(self._device),
            n_refiner_iterations=self._n_refiner_iterations,
        )
        return self._predictions_to_results(predictions, kept)

    # ------------------------------------------------------------------
    # Helpers
    # ------------------------------------------------------------------

    def _predictions_to_results(
        self, predictions: Any, kept: list[MegaPrompt],
    ) -> list[MegaResult]:
        out: list[MegaResult] = []
        infos = getattr(predictions, 'infos', None)
        poses = getattr(predictions, 'poses', None)
        if infos is None or poses is None:
            self._node.get_logger().warn(
                'happypose predictions missing .infos or .poses — nothing to publish'
            )
            return out

        poses_np = poses.detach().cpu().numpy()
        by_label = {p.class_name: p for p in kept}

        for i, row in infos.iterrows():
            label = str(row.get('label', ''))
            score = float(row.get('pose_score', row.get('score', 0.0)))
            if score < self._score_threshold:
                continue
            src = by_label.get(label)
            if src is None:
                continue
            out.append(MegaResult(
                pose_4x4=np.asarray(poses_np[i], dtype=np.float64),
                score=score,
                class_name=label,
                object_id=src.object_id,
            ))
        return out


def _log_happypose_version(node: Node) -> None:
    # Why: happypose is pre-1.0. Log version + install path at load() so
    # API drift surfaces in container logs before inference breaks.
    try:
        out = subprocess.check_output(
            ['pip', 'show', 'happypose'], stderr=subprocess.STDOUT, timeout=5,
        ).decode(errors='replace').strip()
    except Exception as e:  # noqa: BLE001
        node.get_logger().warn(f'pip show happypose failed: {e}')
        return
    node.get_logger().info(f'happypose install info:\n{out}')


def _discover_meshes(mesh_dir: str) -> dict[str, str]:
    """Return {class_name: mesh_path} for every supported file in ``mesh_dir``.

    Class name = file stem, lowercased. Flat layout — nested class
    hierarchies live in the YOLO config, not in the mesh store. This
    matches the FoundationPose mesh registry exactly so meshes can be
    shared between the two backends without duplication.
    """
    found: dict[str, str] = {}
    for entry in sorted(os.listdir(mesh_dir)):
        stem, ext = os.path.splitext(entry)
        if ext.lower() not in _MESH_EXTS:
            continue
        found[stem.lower()] = os.path.join(mesh_dir, entry)
    return found
