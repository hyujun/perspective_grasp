"""BundleSDF unknown-object 6D tracker LifecycleNode.

Pairs SAM2 masks with aligned RGB-D frames (by timestamp), runs the
configured tracker backend once per frame across all masks, and
publishes results as ``PoseWithMetaArray`` on ``/bundlesdf/raw_poses``
(or ``/{cam_ns}/bundlesdf/raw_poses`` under multi-camera fan-out).

Concrete inference lives in ``backends/`` so the node stays framework-
agnostic — ``bundlesdf`` for the Docker service, ``mock`` for CPU-only
smoke tests. The backend contract is the unknown-object tracker variant
of the BasePoseBackend pattern used by SAM2 / FoundationPose / …
(see ``backends/base_backend.py``).
"""

from __future__ import annotations

import importlib
import threading
from collections import deque

import numpy as np
import rclpy
from cv_bridge import CvBridge
from geometry_msgs.msg import PoseStamped
from rclpy.lifecycle import LifecycleNode, LifecycleState, TransitionCallbackReturn
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import CameraInfo, Image

from perception_msgs.msg import (
    PoseWithMeta,
    PoseWithMetaArray,
    SegmentationArray,
)

from bundlesdf_unknown_tracker.backends.base_backend import (
    BaseTrackerBackend,
    TrackPrompt,
    TrackResult,
)


BACKEND_REGISTRY: dict[str, str] = {
    'bundlesdf':
        'bundlesdf_unknown_tracker.backends.bundlesdf_backend'
        ':BundleSdfBackend',
    'mock':
        'bundlesdf_unknown_tracker.backends.mock_backend'
        ':MockTrackerBackend',
}


def _load_backend(key: str, node) -> BaseTrackerBackend:
    if key not in BACKEND_REGISTRY:
        avail = ', '.join(sorted(BACKEND_REGISTRY))
        raise ValueError(f"Unknown backend '{key}'. Available: [{avail}]")
    module_path, class_name = BACKEND_REGISTRY[key].rsplit(':', 1)
    module = importlib.import_module(module_path)
    cls = getattr(module, class_name)
    if not issubclass(cls, BaseTrackerBackend):
        raise TypeError(f'{cls.__name__} is not a BaseTrackerBackend')
    return cls(node)


class BundleSdfNode(LifecycleNode):
    """LifecycleNode wrapping a BundleSDF-style unknown-object tracker."""

    def __init__(self) -> None:
        super().__init__('bundlesdf_tracker')

        self._backend_type: str = (
            self.declare_parameter('backend', 'bundlesdf')
            .get_parameter_value().string_value
        )
        self._image_topic: str = (
            self.declare_parameter('image_topic', '/camera/color/image_raw')
            .get_parameter_value().string_value
        )
        self._depth_topic: str = (
            self.declare_parameter('depth_topic', '/camera/depth/image_rect_raw')
            .get_parameter_value().string_value
        )
        self._camera_info_topic: str = (
            self.declare_parameter('camera_info_topic', '/camera/color/camera_info')
            .get_parameter_value().string_value
        )
        self._masks_topic: str = (
            self.declare_parameter('masks_topic', '/sam2/masks')
            .get_parameter_value().string_value
        )
        self._poses_topic: str = (
            self.declare_parameter('poses_topic', '/bundlesdf/raw_poses')
            .get_parameter_value().string_value
        )
        self._frame_queue_size: int = (
            self.declare_parameter('frame_queue_size', 10)
            .get_parameter_value().integer_value
        )
        self._sync_tolerance_sec: float = (
            self.declare_parameter('sync_tolerance_sec', 0.05)
            .get_parameter_value().double_value
        )
        self._stale_track_frames: int = (
            self.declare_parameter('stale_track_frames', 30)
            .get_parameter_value().integer_value
        )
        self.declare_parameter('min_mask_confidence', 0.0)

        self._bridge = CvBridge()
        self._backend: BaseTrackerBackend | None = None
        self._K: np.ndarray | None = None
        self._frame_lock = threading.Lock()
        # (stamp_ns, rgb, depth_m, frame_id)
        self._frame_queue: deque[tuple[int, np.ndarray, np.ndarray, str]] = deque()
        self._pending_rgb: dict[int, tuple[np.ndarray, str]] = {}
        self._pending_depth: dict[int, np.ndarray] = {}
        self._pending_max = 16

        # Per-track last-seen counter (increments every mask frame; reset
        # when the track is observed). When it crosses stale_track_frames
        # we ask the backend to drop its state for that id.
        self._track_age: dict[int, int] = {}

        self._sub_image = None
        self._sub_depth = None
        self._sub_camera_info = None
        self._sub_masks = None
        self._pub_poses = None
        self._warned_no_camera_info = False

        self.get_logger().info(
            f"BundleSdfNode created (backend='{self._backend_type}')"
        )

    # ------------------------------------------------------------------
    # Lifecycle transitions
    # ------------------------------------------------------------------

    def on_configure(self, state: LifecycleState) -> TransitionCallbackReturn:
        self.get_logger().info('Configuring BundleSdfNode')

        try:
            self._backend = _load_backend(self._backend_type, self)
        except (ValueError, TypeError) as e:
            self.get_logger().error(f'backend load failed: {e}')
            return TransitionCallbackReturn.FAILURE

        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self._sub_image = self.create_subscription(
            Image, self._image_topic, self._image_callback, qos,
        )
        self._sub_depth = self.create_subscription(
            Image, self._depth_topic, self._depth_callback, qos,
        )
        self._sub_camera_info = self.create_subscription(
            CameraInfo, self._camera_info_topic, self._camera_info_callback, qos,
        )
        self._sub_masks = self.create_subscription(
            SegmentationArray, self._masks_topic,
            self._masks_callback, qos,
        )
        self._pub_poses = self.create_lifecycle_publisher(
            PoseWithMetaArray, self._poses_topic, qos,
        )
        self.get_logger().info(
            f'subs: rgb={self._image_topic} depth={self._depth_topic} '
            f'info={self._camera_info_topic} masks={self._masks_topic}  '
            f'pub: {self._poses_topic}'
        )
        return TransitionCallbackReturn.SUCCESS

    def on_activate(self, state: LifecycleState) -> TransitionCallbackReturn:
        self.get_logger().info('Activating (loading backend)')
        assert self._backend is not None
        try:
            self._backend.load()
        except Exception as e:  # noqa: BLE001
            self.get_logger().error(f'backend load failed: {e}')
            return TransitionCallbackReturn.FAILURE
        return super().on_activate(state)

    def on_deactivate(self, state: LifecycleState) -> TransitionCallbackReturn:
        self.get_logger().info('Deactivating (unloading backend)')
        if self._backend is not None:
            self._backend.unload()
        with self._frame_lock:
            self._frame_queue.clear()
            self._pending_rgb.clear()
            self._pending_depth.clear()
        self._track_age.clear()
        return super().on_deactivate(state)

    def on_cleanup(self, state: LifecycleState) -> TransitionCallbackReturn:
        self.get_logger().info('Cleaning up BundleSdfNode')
        if self._backend is not None:
            self._backend.unload()
            self._backend = None
        for sub in (self._sub_image, self._sub_depth,
                    self._sub_camera_info, self._sub_masks):
            if sub is not None:
                self.destroy_subscription(sub)
        if self._pub_poses is not None:
            self.destroy_publisher(self._pub_poses)
        self._sub_image = self._sub_depth = None
        self._sub_camera_info = self._sub_masks = None
        self._pub_poses = None
        self._K = None
        with self._frame_lock:
            self._frame_queue.clear()
            self._pending_rgb.clear()
            self._pending_depth.clear()
        self._track_age.clear()
        return TransitionCallbackReturn.SUCCESS

    # ------------------------------------------------------------------
    # Subscription callbacks
    # ------------------------------------------------------------------

    def _camera_info_callback(self, msg: CameraInfo) -> None:
        self._K = np.array(msg.k, dtype=np.float64).reshape(3, 3)

    def _image_callback(self, msg: Image) -> None:
        try:
            rgb = self._bridge.imgmsg_to_cv2(msg, desired_encoding='rgb8')
        except Exception as e:  # noqa: BLE001
            self.get_logger().warn(f'cv_bridge (rgb) failed: {e}')
            return
        stamp_ns = _stamp_to_ns(msg.header.stamp)
        with self._frame_lock:
            depth = self._pending_depth.pop(stamp_ns, None)
            if depth is not None:
                self._enqueue_locked(stamp_ns, rgb, depth, msg.header.frame_id)
            else:
                self._pending_rgb[stamp_ns] = (rgb, msg.header.frame_id)
                self._prune_pending_locked()

    def _depth_callback(self, msg: Image) -> None:
        depth = _depth_to_metres(self._bridge, msg)
        if depth is None:
            self.get_logger().warn(
                f'unsupported depth encoding {msg.encoding!r}'
            )
            return
        stamp_ns = _stamp_to_ns(msg.header.stamp)
        with self._frame_lock:
            pending = self._pending_rgb.pop(stamp_ns, None)
            if pending is not None:
                rgb, frame_id = pending
                self._enqueue_locked(stamp_ns, rgb, depth, frame_id)
            else:
                self._pending_depth[stamp_ns] = depth
                self._prune_pending_locked()

    def _enqueue_locked(self, stamp_ns: int, rgb: np.ndarray,
                        depth: np.ndarray, frame_id: str) -> None:
        self._frame_queue.append((stamp_ns, rgb, depth, frame_id))
        while len(self._frame_queue) > self._frame_queue_size:
            self._frame_queue.popleft()

    def _prune_pending_locked(self) -> None:
        for pending in (self._pending_rgb, self._pending_depth):
            while len(pending) > self._pending_max:
                oldest = min(pending.keys())
                pending.pop(oldest, None)

    def _masks_callback(self, msg: SegmentationArray) -> None:
        if self._backend is None or not self._backend.loaded:
            return
        if not msg.segmentations:
            self._age_all_tracks(seen=set())
            return
        if self._K is None:
            if not self._warned_no_camera_info:
                self.get_logger().warn(
                    'CameraInfo not yet received — skipping masks'
                )
                self._warned_no_camera_info = True
            return
        if self._pub_poses is None:
            return

        target_ns = _stamp_to_ns(msg.header.stamp)
        paired = self._pair_frame(target_ns)
        if paired is None:
            self.get_logger().debug(
                f'no RGB+depth within {self._sync_tolerance_sec:.3f}s of '
                f'masks at t={target_ns}'
            )
            return
        rgb, depth_m, frame_id = paired

        min_conf = (
            self.get_parameter('min_mask_confidence')
            .get_parameter_value().double_value
        )
        prompts: list[TrackPrompt] = []
        seen_ids: set[int] = set()
        for seg in msg.segmentations:
            if seg.confidence < min_conf:
                continue
            mask = _decode_mask(self._bridge, seg.mask)
            if mask is None:
                continue
            if mask.shape != rgb.shape[:2]:
                self.get_logger().debug(
                    f'mask shape {mask.shape} != rgb {rgb.shape[:2]}, skipping'
                )
                continue
            prompts.append(TrackPrompt(
                track_id=int(seg.id),
                mask=mask,
                class_name='',
                confidence=float(seg.confidence),
            ))
            seen_ids.add(int(seg.id))

        self._age_all_tracks(seen_ids)

        if not prompts:
            return

        try:
            results = self._backend.track(rgb, depth_m, self._K, prompts)
        except Exception as e:  # noqa: BLE001
            self.get_logger().error(f'bundlesdf.track failed: {e}')
            return

        out = PoseWithMetaArray()
        out.header = msg.header
        if not out.header.frame_id:
            out.header.frame_id = frame_id
        for res in results:
            out.poses.append(_to_pose_with_meta(res, out.header))
        self._pub_poses.publish(out)

    # ------------------------------------------------------------------
    # Helpers
    # ------------------------------------------------------------------

    def _pair_frame(
        self, target_ns: int,
    ) -> tuple[np.ndarray, np.ndarray, str] | None:
        tol_ns = int(self._sync_tolerance_sec * 1e9)
        best: tuple[int, np.ndarray, np.ndarray, str] | None = None
        best_diff = tol_ns + 1
        with self._frame_lock:
            for stamp_ns, rgb, depth, frame in self._frame_queue:
                diff = abs(stamp_ns - target_ns)
                if diff < best_diff:
                    best_diff = diff
                    best = (stamp_ns, rgb, depth, frame)
        if best is None or best_diff > tol_ns:
            return None
        return best[1], best[2], best[3]

    def _age_all_tracks(self, seen: set[int]) -> None:
        """Reset age for seen tracks, increment for unseen, drop stale ones.

        Keeps BundleSDF's per-track state bounded when SAM2 stops
        reporting an object (occlusion, track switch). ``seen`` is the
        set of track ids present in the latest mask frame.
        """
        for tid in list(self._track_age):
            if tid in seen:
                self._track_age[tid] = 0
            else:
                self._track_age[tid] += 1
                if self._track_age[tid] > self._stale_track_frames:
                    self._track_age.pop(tid, None)
                    if self._backend is not None:
                        self._backend.drop_track(tid)
                    self.get_logger().info(
                        f'Dropped stale bundlesdf track {tid}'
                    )
        for tid in seen:
            self._track_age.setdefault(tid, 0)


# ---------------------------------------------------------------------------
# Helpers (module-level — no node state, unit-testable)
# ---------------------------------------------------------------------------

def _stamp_to_ns(stamp) -> int:
    return int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)


def _depth_to_metres(bridge: CvBridge, msg: Image) -> np.ndarray | None:
    """Convert a depth Image to float32 metres. 16UC1 → mm, 32FC1 → m."""
    if msg.encoding == '16UC1':
        raw = bridge.imgmsg_to_cv2(msg, desired_encoding='16UC1')
        return (raw.astype(np.float32)) * 1e-3
    if msg.encoding == '32FC1':
        return bridge.imgmsg_to_cv2(msg, desired_encoding='32FC1').astype(np.float32)
    return None


def _decode_mask(bridge: CvBridge, mask_msg: Image) -> np.ndarray | None:
    try:
        raw = bridge.imgmsg_to_cv2(mask_msg, desired_encoding='mono8')
    except Exception:  # noqa: BLE001
        return None
    return raw.astype(bool)


def _to_pose_with_meta(res: TrackResult, header) -> PoseWithMeta:
    pw = PoseWithMeta()
    pw.object_id = int(res.track_id)
    pw.class_name = res.class_name
    pw.source = 'bundlesdf'
    pw.confidence = float(res.score)
    pw.fitness_score = float(res.score)

    ps = PoseStamped()
    ps.header = header
    ps.pose.position.x = float(res.pose_4x4[0, 3])
    ps.pose.position.y = float(res.pose_4x4[1, 3])
    ps.pose.position.z = float(res.pose_4x4[2, 3])
    qx, qy, qz, qw = _rot_to_quat(res.pose_4x4[:3, :3])
    ps.pose.orientation.x = qx
    ps.pose.orientation.y = qy
    ps.pose.orientation.z = qz
    ps.pose.orientation.w = qw
    pw.pose = ps
    return pw


def _rot_to_quat(R: np.ndarray) -> tuple[float, float, float, float]:
    """Convert a 3x3 rotation matrix to a (x, y, z, w) quaternion."""
    m00, m01, m02 = R[0]
    m10, m11, m12 = R[1]
    m20, m21, m22 = R[2]
    trace = m00 + m11 + m22
    if trace > 0.0:
        s = np.sqrt(trace + 1.0) * 2.0
        w = 0.25 * s
        x = (m21 - m12) / s
        y = (m02 - m20) / s
        z = (m10 - m01) / s
    elif (m00 > m11) and (m00 > m22):
        s = np.sqrt(1.0 + m00 - m11 - m22) * 2.0
        w = (m21 - m12) / s
        x = 0.25 * s
        y = (m01 + m10) / s
        z = (m02 + m20) / s
    elif m11 > m22:
        s = np.sqrt(1.0 + m11 - m00 - m22) * 2.0
        w = (m02 - m20) / s
        x = (m01 + m10) / s
        y = 0.25 * s
        z = (m12 + m21) / s
    else:
        s = np.sqrt(1.0 + m22 - m00 - m11) * 2.0
        w = (m10 - m01) / s
        x = (m02 + m20) / s
        y = (m12 + m21) / s
        z = 0.25 * s
    return float(x), float(y), float(z), float(w)


def main(args=None):
    rclpy.init(args=args)
    node = BundleSdfNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
