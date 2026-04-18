"""NVIDIA FoundationPose backend.

Runs inside the ``foundationpose`` Docker service; host installs are
not supported because of PyTorch / kaolin / nvdiffrast coupling. All
heavy imports are deferred to :meth:`load` so the module is safe to
import on CPU-only hosts (e.g. for ``colcon test`` on dev laptops).

Design notes
------------
* **Per-class mesh registry.** The node passes ``mesh_dir``; this
  backend scans for ``<class_name>.(obj|ply|stl)`` at ``load()`` and
  indexes one :class:`FoundationPose` estimator per mesh. A prompt with
  no matching mesh is silently dropped (logged at debug level) so the
  pipeline can run with partial coverage.
* **Register-per-frame.** Initial integration uses ``est.register()``
  on every detection rather than track_one() — this trades speed for
  robustness and removes the need to persist per-object state across
  frames. Upgrading to the track path is a follow-up.
* **Depth units.** FoundationPose wants metres; callers convert 16UC1
  millimetres upstream.
"""

from __future__ import annotations

import os
from typing import TYPE_CHECKING, Any

import numpy as np

from .base_backend import BasePoseBackend, PosePrompt, PoseResult

if TYPE_CHECKING:
    from rclpy.node import Node


_MESH_EXTS = ('.obj', '.ply', '.stl')


class FoundationPoseBackend(BasePoseBackend):
    """Zero-shot 6D pose estimation via NVIDIA FoundationPose."""

    def __init__(self, node: Node) -> None:
        super().__init__(node)

        self._mesh_dir: str = (
            node.declare_parameter('mesh_dir', '')
            .get_parameter_value().string_value
        )
        self._refine_iterations: int = (
            node.declare_parameter('refine_iterations', 5)
            .get_parameter_value().integer_value
        )
        self._score_threshold: float = (
            node.declare_parameter('score_threshold', 0.5)
            .get_parameter_value().double_value
        )
        self._device: str = (
            node.declare_parameter('device', 'cuda')
            .get_parameter_value().string_value
        )
        self._depth_max_m: float = (
            node.declare_parameter('depth_max_m', 2.0)
            .get_parameter_value().double_value
        )

        self._torch: Any = None
        self._estimators: dict[str, Any] = {}
        self._meshes: dict[str, Any] = {}

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

        import torch  # type: ignore
        import trimesh  # type: ignore
        from estimater import FoundationPose  # type: ignore
        from estimater import ScorePredictor, PoseRefinePredictor  # type: ignore

        self._torch = torch
        meshes = _discover_meshes(self._mesh_dir)
        if not meshes:
            raise RuntimeError(
                f"no meshes ({'|'.join(_MESH_EXTS)}) found in {self._mesh_dir}"
            )

        self._node.get_logger().info(
            f'Loading FoundationPose with {len(meshes)} meshes from '
            f'{self._mesh_dir} on {self._device}'
        )
        scorer = ScorePredictor()
        refiner = PoseRefinePredictor()
        for class_name, mesh_path in meshes.items():
            mesh = trimesh.load(mesh_path, force='mesh')
            self._meshes[class_name] = mesh
            self._estimators[class_name] = FoundationPose(
                model_pts=mesh.vertices.copy(),
                model_normals=mesh.vertex_normals.copy(),
                mesh=mesh,
                scorer=scorer,
                refiner=refiner,
            )
            self._node.get_logger().info(
                f'  [{class_name}] mesh={os.path.basename(mesh_path)} '
                f'verts={len(mesh.vertices)}'
            )

        self._loaded = True
        self._node.get_logger().info('FoundationPose loaded')

    def unload(self) -> None:
        self._estimators.clear()
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
        depth_m: np.ndarray,
        K: np.ndarray,
        prompts: list[PosePrompt],
    ) -> list[PoseResult]:
        if not self._loaded:
            raise RuntimeError('FoundationPoseBackend.estimate called before load()')
        if depth_m.shape[:2] != image_rgb.shape[:2]:
            raise ValueError(
                f'depth {depth_m.shape[:2]} != rgb {image_rgb.shape[:2]}'
            )

        h, w = depth_m.shape
        depth = np.where(
            (depth_m > 0.0) & (depth_m <= self._depth_max_m),
            depth_m, 0.0,
        ).astype(np.float32)

        out: list[PoseResult] = []
        for p in prompts:
            est = self._estimators.get(p.class_name)
            if est is None:
                self._node.get_logger().debug(
                    f'no mesh registered for class {p.class_name!r}, skipping'
                )
                continue

            mask = _box_to_mask(p.bbox_xyxy, h, w)
            if mask.sum() == 0:
                continue

            # FoundationPose.register returns a (4, 4) camera-from-object pose.
            T = est.register(
                K=K.astype(np.float64),
                rgb=image_rgb,
                depth=depth,
                ob_mask=mask,
                iteration=self._refine_iterations,
            )
            if T is None:
                continue

            score = float(getattr(est, 'best_score', p.confidence))
            if score < self._score_threshold:
                continue

            out.append(PoseResult(
                pose_4x4=np.asarray(T, dtype=np.float64),
                score=score,
                class_name=p.class_name,
                object_id=p.object_id,
            ))
        return out


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

def _discover_meshes(mesh_dir: str) -> dict[str, str]:
    """Return {class_name: mesh_path} for every supported file in ``mesh_dir``.

    Class name = file stem, lowercased. Directory layout is flat by
    design — nested class hierarchies live in the YOLO config, not in
    the mesh store.
    """
    found: dict[str, str] = {}
    for entry in sorted(os.listdir(mesh_dir)):
        stem, ext = os.path.splitext(entry)
        if ext.lower() not in _MESH_EXTS:
            continue
        found[stem.lower()] = os.path.join(mesh_dir, entry)
    return found


def _box_to_mask(box_xyxy: tuple[float, float, float, float],
                 h: int, w: int) -> np.ndarray:
    """Return a bool mask with the bbox rectangle set to True."""
    x1, y1, x2, y2 = box_xyxy
    x1 = int(np.clip(x1, 0, w - 1))
    y1 = int(np.clip(y1, 0, h - 1))
    x2 = int(np.clip(x2, 0, w))
    y2 = int(np.clip(y2, 0, h))
    mask = np.zeros((h, w), dtype=bool)
    if x2 > x1 and y2 > y1:
        mask[y1:y2, x1:x2] = True
    return mask
