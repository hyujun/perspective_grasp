"""MegaPose zero-shot RGB 6D pose estimation via happypose.

Runs inside the shared ``cosypose-runtime`` Docker stage (same
happypose install). Heavy imports live in :meth:`load` so the module
is import-safe on CPU-only hosts.

Design notes
------------
* **Mesh registry from disk.** Unlike CosyPose (dataset-driven),
  MegaPose is zero-shot and can accept any mesh at inference — we
  scan ``mesh_dir`` for ``<class_name>.(obj|ply)`` at load time and
  build a happypose ``RigidObjectDataset`` that the model uses for
  rendering. A prompt whose class has no mesh is silently dropped
  (logged at debug level).
* **Named-model driven.** happypose ships a ``NAMED_MODELS`` dict
  keyed by model string (``megapose-1.0-RGB``,
  ``megapose-1.0-RGB-multi-hypothesis``, …). Each entry bundles the
  coarse/refiner run ids plus the ``inference_parameters`` dict we
  pass straight through to ``run_inference_pipeline``. No hand-
  rolled iteration tuning — the upstream dict is the source of truth.
* **API surface (verified against agimus-project/happypose @main,
  2026-04):** ``happypose.toolbox.utils.load_model`` (not the
  ``pose_estimators.megapose.inference`` path our first cut guessed)
  owns both ``load_named_model`` and ``NAMED_MODELS``. The
  ``PoseEstimator`` it returns is already ``.to(device)``-ready.
  ``_log_happypose_version`` at load() pins the exact commit in
  container logs so future drift surfaces loudly.
"""

from __future__ import annotations

import os
import subprocess
from typing import TYPE_CHECKING, Any

import numpy as np

from .base_backend import BaseMegaBackend, MegaPrompt, MegaResult

if TYPE_CHECKING:
    from rclpy.node import Node


_MESH_EXTS = ('.obj', '.ply')


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
        self._mesh_units: str = (
            node.declare_parameter('mesh_units', 'mm')
            .get_parameter_value().string_value
        )
        self._score_threshold: float = (
            node.declare_parameter('score_threshold', 0.3)
            .get_parameter_value().double_value
        )
        # Stored as-is here; the actual probe + fallback runs in load()
        # so importing this module on CPU-only hosts (colcon test) never
        # reaches torch. 'auto' = cuda if usable else cpu.
        self._requested_device: str = (
            node.declare_parameter('device', 'auto')
            .get_parameter_value().string_value
        )
        self._device: str = ''  # populated by load() via resolve_torch_device
        # happypose load_named_model defaults: n_workers=4, bsz_images=128.
        # Surfaced here for 8GB VRAM tuning without a container rebuild.
        self._n_workers: int = (
            node.declare_parameter('n_workers', 4)
            .get_parameter_value().integer_value
        )
        self._bsz_images: int = (
            node.declare_parameter('bsz_images', 128)
            .get_parameter_value().integer_value
        )

        self._torch: Any = None
        self._estimator: Any = None
        self._observation_cls: Any = None
        self._make_detections_fn: Any = None
        self._object_data_cls: Any = None
        self._inference_params: dict[str, Any] = {}
        self._known_classes: set[str] = set()

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
        from happypose.toolbox.datasets.object_dataset import (  # type: ignore
            RigidObject, RigidObjectDataset,
        )
        from happypose.toolbox.datasets.scene_dataset import ObjectData  # type: ignore
        from happypose.toolbox.inference.types import (  # type: ignore
            ObservationTensor,
        )
        from happypose.toolbox.inference.utils import (  # type: ignore
            make_detections_from_object_data,
        )
        from happypose.toolbox.utils.load_model import (  # type: ignore
            NAMED_MODELS, load_named_model,
        )
        from perception_launch_utils import resolve_torch_device

        self._torch = torch
        self._device = resolve_torch_device(
            self._requested_device, self._node.get_logger(), torch_mod=torch,
        ).device
        self._observation_cls = ObservationTensor
        self._make_detections_fn = make_detections_from_object_data
        self._object_data_cls = ObjectData

        if self._model_name not in NAMED_MODELS:
            raise RuntimeError(
                f"model_name '{self._model_name}' not in happypose NAMED_MODELS "
                f"(available: {sorted(NAMED_MODELS)})"
            )
        named = NAMED_MODELS[self._model_name]
        # happypose marks RGBD / ICP-refined variants with requires_depth=True.
        # This wrapper consumes RGB only — fail fast at load() with a clear
        # message rather than crash inside inference on a missing depth tensor.
        if named.get('requires_depth', False):
            raise RuntimeError(
                f"model_name '{self._model_name}' requires depth input, but "
                f"this wrapper is RGB-only. Pick an RGB-only NAMED_MODELS "
                f"entry (e.g. 'megapose-1.0-RGB-multi-hypothesis')."
            )
        self._inference_params = dict(named['inference_parameters'])

        rigid_objects, labels = _build_rigid_objects(
            self._mesh_dir, self._mesh_units, RigidObject,
        )
        if not rigid_objects:
            raise RuntimeError(
                f"no .obj/.ply meshes found in {self._mesh_dir}"
            )
        self._known_classes = labels
        object_dataset = RigidObjectDataset(rigid_objects)

        self._node.get_logger().info(
            f'Loading MegaPose model={self._model_name} '
            f'meshes={len(rigid_objects)} on {self._device}'
        )
        self._estimator = load_named_model(
            self._model_name, object_dataset,
            n_workers=self._n_workers,
            bsz_images=self._bsz_images,
        ).to(self._device)

        self._loaded = True
        self._node.get_logger().info('MegaPose loaded')

    def unload(self) -> None:
        self._estimator = None
        self._known_classes = set()
        self._inference_params = {}
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
        object_data: list[Any] = []
        for p in prompts:
            if p.class_name not in self._known_classes:
                self._node.get_logger().debug(
                    f'no mesh registered for class {p.class_name!r}, skipping'
                )
                continue
            kept.append(p)
            object_data.append(self._object_data_cls(
                label=p.class_name,
                bbox_modal=np.asarray(p.bbox_xyxy, dtype=np.float32),
            ))
        if not kept:
            return []

        detections_tensor = self._make_detections_fn(object_data)
        obs = self._observation_cls.from_numpy(
            rgb=image_rgb, K=np.asarray(K, dtype=np.float64),
        )
        predictions, extras = self._estimator.run_inference_pipeline(
            observation=obs.to(self._device),
            detections=detections_tensor.to(self._device),
            run_detector=False,
            **self._inference_params,
        )

        timing = extras.get('timing_str') if isinstance(extras, dict) else None
        if timing:
            self._node.get_logger().debug(f'megapose timings: {timing}')

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


def _build_rigid_objects(
    mesh_dir: str, mesh_units: str, rigid_object_cls: Any,
) -> tuple[list[Any], set[str]]:
    """Build happypose ``RigidObject`` entries from a flat mesh dir.

    Panda3dBatchRenderer accepts .obj/.ply only; .stl is silently
    ignored. Label = file stem (lowercased) so the set matches YOLO
    class names exactly and is shareable with the CosyPose backend.
    """
    import pathlib
    rigid_objects: list[Any] = []
    labels: set[str] = set()
    for entry in sorted(os.listdir(mesh_dir)):
        stem, ext = os.path.splitext(entry)
        if ext.lower() not in _MESH_EXTS:
            continue
        label = stem.lower()
        rigid_objects.append(rigid_object_cls(
            label=label,
            mesh_path=pathlib.Path(mesh_dir) / entry,
            mesh_units=mesh_units,
        ))
        labels.add(label)
    return rigid_objects, labels
