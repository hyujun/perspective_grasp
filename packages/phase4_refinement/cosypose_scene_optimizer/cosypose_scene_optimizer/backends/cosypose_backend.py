"""CosyPose scene-level pose estimation via happypose.

Runs inside the ``cosypose`` Docker service; host installs are not
supported because of the happypose + pytorch3d coupling. All heavy
imports are deferred to :meth:`load` so the module is safe to import
on CPU-only hosts (colcon test, dev laptops).

Design notes
------------
* **Pre-computed detections.** We pass YOLO detections through as
  happypose ``DetectionsType`` — no need to run a happypose detector.
* **Mesh-file-driven registry.** ``mesh_dir`` is scanned at load()
  for ``<label>.(obj|ply)``; each becomes a ``RigidObject`` entry in
  the happypose ``RigidObjectDataset`` that drives both rendering
  and mesh-db construction. This matches the MegaPose backend layout
  exactly so meshes can be shared between the two nodes without
  duplication. A prompt whose class name has no mesh is silently
  dropped (logged at debug level).
* **Coarse + refine iterations.** Parametrised so tuning does not
  require a container rebuild. Defaults match happypose CosyPose
  recommended config (1 coarse / 4 refine).
* **API surface (verified against agimus-project/happypose @main,
  2026-04):** ``happypose.toolbox.inference.utils.load_pose_models``
  and ``happypose.pose_estimators.cosypose.cosypose.integrated
  .pose_estimator.PoseEstimator`` are the real entry points.
  ``_log_happypose_version`` at load() pins the exact commit in
  container logs so future drift surfaces loudly.
"""

from __future__ import annotations

import os
import subprocess
from typing import TYPE_CHECKING, Any

import numpy as np

from .base_backend import BaseSceneBackend, SceneDetection, SceneResult

if TYPE_CHECKING:
    from rclpy.node import Node


class CosyPoseBackend(BaseSceneBackend):
    """Multi-object scene solve via happypose/CosyPose."""

    def __init__(self, node: Node) -> None:
        super().__init__(node)

        self._dataset_dir: str = (
            node.declare_parameter('dataset_dir', '')
            .get_parameter_value().string_value
        )
        self._dataset_name: str = (
            node.declare_parameter('dataset_name', 'ycbv')
            .get_parameter_value().string_value
        )
        self._mesh_dir: str = (
            node.declare_parameter('mesh_dir', '')
            .get_parameter_value().string_value
        )
        self._mesh_units: str = (
            node.declare_parameter('mesh_units', 'mm')
            .get_parameter_value().string_value
        )
        self._coarse_run_id: str = (
            node.declare_parameter('coarse_run_id', '')
            .get_parameter_value().string_value
        )
        self._refiner_run_id: str = (
            node.declare_parameter('refiner_run_id', '')
            .get_parameter_value().string_value
        )
        self._n_coarse_iterations: int = (
            node.declare_parameter('n_coarse_iterations', 1)
            .get_parameter_value().integer_value
        )
        self._n_refiner_iterations: int = (
            node.declare_parameter('n_refiner_iterations', 4)
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
        self._object_data_cls: Any = None
        self._known_classes: set[str] = set()

    # ------------------------------------------------------------------
    # Lifecycle
    # ------------------------------------------------------------------

    def load(self) -> None:
        if self._loaded:
            return
        if not self._dataset_dir or not os.path.isdir(self._dataset_dir):
            raise RuntimeError(
                f"dataset_dir '{self._dataset_dir}' is missing or not a directory"
            )
        if not self._mesh_dir or not os.path.isdir(self._mesh_dir):
            raise RuntimeError(
                f"mesh_dir '{self._mesh_dir}' is missing or not a directory"
            )
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
            load_pose_models,
            make_detections_from_object_data,
        )
        from happypose.pose_estimators.cosypose.cosypose.integrated.pose_estimator import (  # type: ignore
            PoseEstimator,
        )

        self._torch = torch
        self._observation_cls = ObservationTensor
        self._make_detections_fn = make_detections_from_object_data
        self._object_data_cls = ObjectData

        rigid_objects, mesh_labels = _build_rigid_objects(
            self._mesh_dir, self._mesh_units, RigidObject,
        )
        if not rigid_objects:
            raise RuntimeError(
                f"no .obj/.ply meshes found in {self._mesh_dir}"
            )
        object_dataset = RigidObjectDataset(rigid_objects)

        coarse_run = self._coarse_run_id or f'coarse-bop-{self._dataset_name}-pbr'
        refiner_run = self._refiner_run_id or f'refiner-bop-{self._dataset_name}-pbr'
        self._node.get_logger().info(
            f'Loading CosyPose coarse={coarse_run} refiner={refiner_run} '
            f'meshes={len(rigid_objects)} on {self._device}'
        )

        coarse_model, refiner_model, _mesh_db = load_pose_models(
            coarse_run_id=coarse_run,
            refiner_run_id=refiner_run,
            object_dataset=object_dataset,
            force_panda3d_renderer=True,
        )
        self._estimator = PoseEstimator(
            refiner_model=refiner_model,
            coarse_model=coarse_model,
            detector_model=None,
        )
        self._known_classes = mesh_labels

        self._loaded = True
        self._node.get_logger().info(
            f'CosyPose loaded ({len(self._known_classes)} known classes)'
        )

    def unload(self) -> None:
        self._estimator = None
        self._known_classes = set()
        if self._torch is not None:
            try:
                self._torch.cuda.empty_cache()
            except Exception:  # noqa: BLE001
                pass
        self._loaded = False

    # ------------------------------------------------------------------
    # Inference
    # ------------------------------------------------------------------

    def optimize(
        self,
        image_rgb: np.ndarray,
        K: np.ndarray,
        detections: list[SceneDetection],
        target_classes: list[str] | None = None,
    ) -> list[SceneResult]:
        if not self._loaded or self._estimator is None:
            raise RuntimeError('CosyPoseBackend.optimize called before load()')

        kept, object_data = self._filter_prompts(detections, target_classes)
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
            n_coarse_iterations=self._n_coarse_iterations,
            n_refiner_iterations=self._n_refiner_iterations,
        )

        timing = extras.get('timing_str') if isinstance(extras, dict) else None
        if timing:
            self._node.get_logger().debug(f'cosypose timings: {timing}')

        return self._predictions_to_results(predictions, kept)

    # ------------------------------------------------------------------
    # Helpers
    # ------------------------------------------------------------------

    def _filter_prompts(
        self, detections: list[SceneDetection],
        target_classes: list[str] | None,
    ) -> tuple[list[SceneDetection], list[Any]]:
        kept: list[SceneDetection] = []
        object_data: list[Any] = []
        for d in detections:
            if target_classes and d.class_name not in target_classes:
                continue
            if self._known_classes and d.class_name not in self._known_classes:
                self._node.get_logger().debug(
                    f'class {d.class_name!r} not in CosyPose registry, skipping'
                )
                continue
            kept.append(d)
            object_data.append(self._object_data_cls(
                label=d.class_name,
                bbox_modal=np.asarray(d.bbox_xyxy, dtype=np.float32),
            ))
        return kept, object_data

    def _predictions_to_results(
        self, predictions: Any, kept: list[SceneDetection],
    ) -> list[SceneResult]:
        out: list[SceneResult] = []
        infos = getattr(predictions, 'infos', None)
        poses = getattr(predictions, 'poses', None)
        if infos is None or poses is None:
            self._node.get_logger().warn(
                'happypose predictions missing .infos or .poses — nothing to publish'
            )
            return out

        poses_np = poses.detach().cpu().numpy()
        by_label = {d.class_name: d for d in kept}

        for i, row in infos.iterrows():
            label = str(row.get('label', ''))
            score = float(row.get('pose_score', row.get('score', 0.0)))
            if score < self._score_threshold:
                continue
            src = by_label.get(label)
            if src is None:
                continue
            out.append(SceneResult(
                pose_4x4=np.asarray(poses_np[i], dtype=np.float64),
                score=score,
                class_name=label,
                object_id=src.object_id,
            ))
        return out


_MESH_EXTS = ('.obj', '.ply')


def _build_rigid_objects(
    mesh_dir: str, mesh_units: str, rigid_object_cls: Any,
) -> tuple[list[Any], set[str]]:
    # load_pose_models + Panda3dBatchRenderer accept .obj/.ply only;
    # .stl is silently ignored. Label = file stem (lowercased) so the
    # set matches YOLO class names exactly.
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
