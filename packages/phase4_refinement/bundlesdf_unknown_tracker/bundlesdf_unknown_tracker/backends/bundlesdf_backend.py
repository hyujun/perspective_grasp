"""NVlabs/BundleSDF backend.

Runs inside the ``bundlesdf`` Docker service; host installs are not
supported because of BundleSDF's PyTorch / PyTorch3D / kaolin / custom
CUDA op coupling. All heavy imports are deferred to :meth:`load` so
this module is safe to import on CPU-only hosts (e.g. for
``colcon test`` on dev laptops).

Design notes
------------
* **Per-track BundleSdf instances.** BundleSDF builds a neural SDF +
  pose graph for one object at a time. We hold a dict
  ``track_id → BundleSdf`` and dispatch ``run()`` per prompt, lazily
  instantiating a new tracker the first time a ``track_id`` is seen.
* **Config files.** NVlabs ships ``BundleTrack/config_ho3d.yml`` and
  ``nerf_runner.yml`` with the repo; paths are node parameters so a
  user can override with a tuned set mounted via the weights volume.
* **Out dir.** BundleSDF writes keyframes / debug renders to
  ``out_folder``; we create one sub-dir per track so concurrent
  trackers don't clobber each other.
* **API probe.** ``load()`` logs ``pip show bundlesdf`` + the real
  ``BundleSdf.run`` signature — any upstream API drift surfaces in
  container logs on first activate, mirroring ``cosypose_backend``.
"""

from __future__ import annotations

import importlib
import importlib.metadata as importlib_metadata
import inspect
import os
from typing import TYPE_CHECKING, Any

import numpy as np

from .base_backend import BaseTrackerBackend, TrackPrompt, TrackResult

if TYPE_CHECKING:
    from rclpy.node import Node


_BUNDLESDF_SRC = '/opt/BundleSDF'


class BundleSdfBackend(BaseTrackerBackend):
    """Incremental unknown-object 6D tracker via NVlabs BundleSDF."""

    def __init__(self, node: Node) -> None:
        super().__init__(node)

        self._bundlesdf_src: str = (
            node.declare_parameter('bundlesdf_src', _BUNDLESDF_SRC)
            .get_parameter_value().string_value
        )
        self._cfg_track_path: str = (
            node.declare_parameter(
                'cfg_track_path',
                os.path.join(_BUNDLESDF_SRC, 'BundleTrack', 'config_ho3d.yml'),
            ).get_parameter_value().string_value
        )
        self._cfg_nerf_path: str = (
            node.declare_parameter(
                'cfg_nerf_path',
                os.path.join(_BUNDLESDF_SRC, 'nerf_runner.yml'),
            ).get_parameter_value().string_value
        )
        self._out_dir: str = (
            node.declare_parameter('out_dir', '/ws/models/bundlesdf/out')
            .get_parameter_value().string_value
        )
        self._shorter_side: int = (
            node.declare_parameter('shorter_side', 480)
            .get_parameter_value().integer_value
        )
        self._depth_max_m: float = (
            node.declare_parameter('depth_max_m', 2.0)
            .get_parameter_value().double_value
        )
        self._min_mask_area: int = (
            node.declare_parameter('min_mask_area', 400)
            .get_parameter_value().integer_value
        )
        # Stored as-is here; the actual probe + fallback runs in load()
        # so importing this module on CPU-only hosts (colcon test) never
        # reaches torch. 'auto' = cuda if usable else cpu.
        self._requested_device: str = (
            node.declare_parameter('device', 'auto')
            .get_parameter_value().string_value
        )
        self._device: str = ''  # populated by load() via resolve_torch_device

        self._torch: Any = None
        self._BundleSdf: Any = None
        self._trackers: dict[int, Any] = {}
        # Monotonic frame counter per track — BundleSDF wants a unique
        # id_str per call so internal keyframe bookkeeping lines up.
        self._frame_idx: dict[int, int] = {}

    # ------------------------------------------------------------------
    # Lifecycle
    # ------------------------------------------------------------------

    def load(self) -> None:
        if self._loaded:
            return

        if self._bundlesdf_src and self._bundlesdf_src not in os.sys.path:
            os.sys.path.insert(0, self._bundlesdf_src)

        import torch  # type: ignore

        try:
            bundlesdf_mod = importlib.import_module('bundlesdf')
        except ImportError as e:
            raise RuntimeError(
                f'import bundlesdf failed from {self._bundlesdf_src!r}: {e}'
            ) from e
        BundleSdf = getattr(bundlesdf_mod, 'BundleSdf', None)
        if BundleSdf is None:
            raise RuntimeError(
                f'{bundlesdf_mod!r} does not export BundleSdf — upstream '
                'may have renamed the top-level class'
            )

        from perception_launch_utils import resolve_torch_device

        self._torch = torch
        self._device = resolve_torch_device(
            self._requested_device, self._node.get_logger(), torch_mod=torch,
        ).device
        self._BundleSdf = BundleSdf

        for path in (self._cfg_track_path, self._cfg_nerf_path):
            if not os.path.isfile(path):
                raise RuntimeError(f'BundleSDF config missing: {path}')
        os.makedirs(self._out_dir, exist_ok=True)

        self._log_versions(bundlesdf_mod)
        self._loaded = True
        self._node.get_logger().info(
            f'BundleSDF loaded (src={self._bundlesdf_src}, device={self._device})'
        )

    def unload(self) -> None:
        self._trackers.clear()
        self._frame_idx.clear()
        if self._torch is not None:
            try:
                self._torch.cuda.empty_cache()
            except Exception:  # noqa: BLE001
                pass
        self._loaded = False

    def drop_track(self, track_id: int) -> None:
        self._trackers.pop(track_id, None)
        self._frame_idx.pop(track_id, None)

    # ------------------------------------------------------------------
    # Inference
    # ------------------------------------------------------------------

    def track(
        self,
        image_rgb: np.ndarray,
        depth_m: np.ndarray,
        K: np.ndarray,
        prompts: list[TrackPrompt],
    ) -> list[TrackResult]:
        if not self._loaded:
            raise RuntimeError('BundleSdfBackend.track called before load()')
        if depth_m.shape[:2] != image_rgb.shape[:2]:
            raise ValueError(
                f'depth {depth_m.shape[:2]} != rgb {image_rgb.shape[:2]}'
            )

        h, w = depth_m.shape
        depth = np.where(
            (depth_m > 0.0) & (depth_m <= self._depth_max_m),
            depth_m, 0.0,
        ).astype(np.float32)

        out: list[TrackResult] = []
        for p in prompts:
            if p.mask.shape != (h, w):
                continue
            if int(p.mask.sum()) < self._min_mask_area:
                continue

            tracker = self._get_or_create_tracker(p.track_id)
            if tracker is None:
                continue

            idx = self._frame_idx.get(p.track_id, 0)
            id_str = f'{p.track_id:05d}_{idx:06d}'
            self._frame_idx[p.track_id] = idx + 1

            try:
                pose = tracker.run(
                    color=image_rgb,
                    depth=depth,
                    K=K.astype(np.float64),
                    id_str=id_str,
                    mask=p.mask.astype(np.uint8) * 255,
                    occ_mask=None,
                )
            except Exception as e:  # noqa: BLE001
                self._node.get_logger().warn(
                    f'BundleSDF.run failed for track {p.track_id}: {e}'
                )
                continue

            pose_arr = _as_pose_4x4(pose)
            if pose_arr is None:
                continue

            out.append(TrackResult(
                pose_4x4=pose_arr,
                score=float(p.confidence),
                track_id=p.track_id,
                class_name=p.class_name,
            ))
        return out

    # ------------------------------------------------------------------
    # Helpers
    # ------------------------------------------------------------------

    def _get_or_create_tracker(self, track_id: int) -> Any | None:
        if track_id in self._trackers:
            return self._trackers[track_id]

        out_folder = os.path.join(self._out_dir, f'track_{track_id:05d}')
        os.makedirs(out_folder, exist_ok=True)
        try:
            tracker = self._BundleSdf(
                cfg_track_dir=self._cfg_track_path,
                cfg_nerf_dir=self._cfg_nerf_path,
                start_nerf_keyframes=5,
                translation=np.zeros(3, dtype=np.float64),
                shorter_side=self._shorter_side,
                out_folder=out_folder,
                use_segmenter=False,
                use_gui=False,
            )
        except TypeError:
            # Older API — fall back to positional / minimal constructor.
            try:
                tracker = self._BundleSdf(
                    self._cfg_track_path,
                    self._cfg_nerf_path,
                    out_folder,
                )
            except Exception as e:  # noqa: BLE001
                self._node.get_logger().error(
                    f'BundleSdf constructor failed for track {track_id}: {e}'
                )
                return None
        except Exception as e:  # noqa: BLE001
            self._node.get_logger().error(
                f'BundleSdf constructor failed for track {track_id}: {e}'
            )
            return None

        self._trackers[track_id] = tracker
        self._node.get_logger().info(
            f'BundleSDF tracker initialised for track_id={track_id} '
            f'(out={out_folder})'
        )
        return tracker

    def _log_versions(self, bundlesdf_mod: Any) -> None:
        logger = self._node.get_logger()
        try:
            meta = importlib_metadata.metadata('bundlesdf')
            logger.info(f"pip show bundlesdf: version={meta.get('Version')}")
        except importlib_metadata.PackageNotFoundError:
            logger.info(
                'bundlesdf is not pip-registered (loaded from source tree); '
                f'module file: {getattr(bundlesdf_mod, "__file__", "?")}'
            )
        except Exception as e:  # noqa: BLE001
            logger.warn(f'importlib.metadata(bundlesdf) failed: {e}')

        try:
            sig = inspect.signature(self._BundleSdf.__init__)
            logger.info(f'BundleSdf.__init__ signature: {sig}')
        except (TypeError, ValueError) as e:
            logger.warn(f'inspect BundleSdf.__init__ failed: {e}')
        try:
            sig = inspect.signature(self._BundleSdf.run)
            logger.info(f'BundleSdf.run signature: {sig}')
        except (TypeError, ValueError) as e:
            logger.warn(f'inspect BundleSdf.run failed: {e}')


def _as_pose_4x4(pose: Any) -> np.ndarray | None:
    """Coerce BundleSDF's return value into a (4, 4) float64 ndarray.

    Upstream has returned ``np.ndarray`` in some versions and torch
    tensors in others; we accept both.
    """
    if pose is None:
        return None
    try:
        arr = np.asarray(pose, dtype=np.float64)
    except Exception:  # noqa: BLE001
        if hasattr(pose, 'detach'):
            arr = pose.detach().cpu().numpy().astype(np.float64)
        else:
            return None
    if arr.shape != (4, 4):
        return None
    return arr
