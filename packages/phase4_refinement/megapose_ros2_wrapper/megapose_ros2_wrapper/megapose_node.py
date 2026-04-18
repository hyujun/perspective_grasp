"""MegaPose zero-shot 6D pose estimation LifecycleNode.

Pairs YOLO detections with their source RGB frame (by timestamp),
runs the configured zero-shot pose backend using its per-class mesh
registry, and publishes results as ``PoseWithMetaArray`` on
``/megapose/raw_poses`` (or ``/{cam_ns}/megapose/raw_poses`` under
multi-camera fan-out).

Concrete inference lives in ``backends/`` — ``megapose`` for the
Docker service, ``mock`` for CPU-only smoke tests. The backend
contract is shaped like FoundationPose's but RGB-only (no depth).
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
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CameraInfo, Image, RegionOfInterest

from perception_msgs.msg import (
    DetectionArray,
    PoseWithMeta,
    PoseWithMetaArray,
)

from megapose_ros2_wrapper.backends.base_backend import (
    BaseMegaBackend,
    MegaPrompt,
    MegaResult,
)


BACKEND_REGISTRY: dict[str, str] = {
    'megapose':
        'megapose_ros2_wrapper.backends.megapose_backend:MegaPoseBackend',
    'mock':
        'megapose_ros2_wrapper.backends.mock_backend:MockMegaBackend',
}


def _load_backend(key: str, node) -> BaseMegaBackend:
    if key not in BACKEND_REGISTRY:
        avail = ', '.join(sorted(BACKEND_REGISTRY))
        raise ValueError(f"Unknown backend '{key}'. Available: [{avail}]")
    module_path, class_name = BACKEND_REGISTRY[key].rsplit(':', 1)
    module = importlib.import_module(module_path)
    cls = getattr(module, class_name)
    if not issubclass(cls, BaseMegaBackend):
        raise TypeError(f'{cls.__name__} is not a BaseMegaBackend')
    return cls(node)


class MegaPoseNode(LifecycleNode):
    """LifecycleNode wrapping an RGB zero-shot 6D pose backend."""

    def __init__(self) -> None:
        super().__init__('megapose_tracker')

        self._backend_type: str = (
            self.declare_parameter('backend', 'megapose')
            .get_parameter_value().string_value
        )
        self._image_topic: str = (
            self.declare_parameter('image_topic', '/camera/color/image_raw')
            .get_parameter_value().string_value
        )
        self._camera_info_topic: str = (
            self.declare_parameter('camera_info_topic', '/camera/color/camera_info')
            .get_parameter_value().string_value
        )
        self._detections_topic: str = (
            self.declare_parameter('detections_topic', '/yolo/detections')
            .get_parameter_value().string_value
        )
        self._poses_topic: str = (
            self.declare_parameter('poses_topic', '/megapose/raw_poses')
            .get_parameter_value().string_value
        )
        self._image_queue_size: int = (
            self.declare_parameter('image_queue_size', 10)
            .get_parameter_value().integer_value
        )
        self._sync_tolerance_sec: float = (
            self.declare_parameter('sync_tolerance_sec', 0.05)
            .get_parameter_value().double_value
        )
        # Re-read every detection callback so ``ros2 param set`` sticks.
        self.declare_parameter('min_detection_confidence', 0.0)

        self._bridge = CvBridge()
        self._backend: BaseMegaBackend | None = None
        self._K: np.ndarray | None = None
        self._image_lock = threading.Lock()
        self._image_queue: deque[tuple[int, np.ndarray, str]] = deque()

        self._sub_image = None
        self._sub_camera_info = None
        self._sub_detections = None
        self._pub_poses = None
        self._warned_no_camera_info = False

        self.get_logger().info(
            f"MegaPoseNode created (backend='{self._backend_type}')"
        )

    # ------------------------------------------------------------------
    # Lifecycle transitions
    # ------------------------------------------------------------------

    def on_configure(self, state: LifecycleState) -> TransitionCallbackReturn:
        self.get_logger().info('Configuring MegaPoseNode')

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
        self._sub_camera_info = self.create_subscription(
            CameraInfo, self._camera_info_topic,
            self._camera_info_callback, qos,
        )
        self._sub_detections = self.create_subscription(
            DetectionArray, self._detections_topic,
            self._detections_callback, qos,
        )
        self._pub_poses = self.create_publisher(
            PoseWithMetaArray, self._poses_topic, qos,
        )
        self.get_logger().info(
            f'subs: rgb={self._image_topic} info={self._camera_info_topic} '
            f'dets={self._detections_topic}  pub: {self._poses_topic}'
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
        with self._image_lock:
            self._image_queue.clear()
        return super().on_deactivate(state)

    def on_cleanup(self, state: LifecycleState) -> TransitionCallbackReturn:
        self.get_logger().info('Cleaning up MegaPoseNode')
        if self._backend is not None:
            self._backend.unload()
            self._backend = None
        for sub in (self._sub_image, self._sub_camera_info, self._sub_detections):
            if sub is not None:
                self.destroy_subscription(sub)
        if self._pub_poses is not None:
            self.destroy_publisher(self._pub_poses)
        self._sub_image = self._sub_camera_info = self._sub_detections = None
        self._pub_poses = None
        self._K = None
        with self._image_lock:
            self._image_queue.clear()
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
            self.get_logger().warn(f'cv_bridge failed: {e}')
            return
        stamp_ns = _stamp_to_ns(msg.header.stamp)
        with self._image_lock:
            self._image_queue.append((stamp_ns, rgb, msg.header.frame_id))
            while len(self._image_queue) > self._image_queue_size:
                self._image_queue.popleft()

    def _detections_callback(self, msg: DetectionArray) -> None:
        if self._backend is None or not self._backend.loaded:
            return
        if not msg.detections:
            return
        if self._K is None:
            if not self._warned_no_camera_info:
                self.get_logger().warn(
                    'CameraInfo not yet received — skipping detections'
                )
                self._warned_no_camera_info = True
            return
        if self._pub_poses is None:
            return

        target_ns = _stamp_to_ns(msg.header.stamp)
        paired = self._pair_image(target_ns)
        if paired is None:
            self.get_logger().debug(
                f'no RGB within {self._sync_tolerance_sec:.3f}s of '
                f'detections at t={target_ns}'
            )
            return
        rgb, frame_id = paired

        min_conf = (
            self.get_parameter('min_detection_confidence')
            .get_parameter_value().double_value
        )
        prompts: list[MegaPrompt] = []
        h, w = rgb.shape[:2]
        for det in msg.detections:
            if det.confidence < min_conf:
                continue
            box = _roi_to_xyxy(det.bbox, w, h)
            if box is None:
                continue
            prompts.append(MegaPrompt(
                object_id=int(det.id),
                class_name=det.class_name,
                bbox_xyxy=box,
                confidence=float(det.confidence),
            ))

        if not prompts:
            return

        try:
            results = self._backend.estimate(rgb, self._K, prompts)
        except Exception as e:  # noqa: BLE001
            self.get_logger().error(f'pose estimation failed: {e}')
            return

        out = PoseWithMetaArray()
        out.header = msg.header
        if not out.header.frame_id:
            out.header.frame_id = frame_id

        for res in results:
            out.poses.append(_to_pose_with_meta(res, out.header))

        self._pub_poses.publish(out)

    # ------------------------------------------------------------------
    # Image / detection pairing
    # ------------------------------------------------------------------

    def _pair_image(
        self, target_ns: int,
    ) -> tuple[np.ndarray, str] | None:
        tol_ns = int(self._sync_tolerance_sec * 1e9)
        best: tuple[int, np.ndarray, str] | None = None
        best_diff = tol_ns + 1
        with self._image_lock:
            for stamp_ns, rgb, frame in self._image_queue:
                diff = abs(stamp_ns - target_ns)
                if diff < best_diff:
                    best_diff = diff
                    best = (stamp_ns, rgb, frame)
        if best is None or best_diff > tol_ns:
            return None
        return best[1], best[2]


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

def _stamp_to_ns(stamp) -> int:
    return int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)


def _roi_to_xyxy(
    roi: RegionOfInterest, width: int, height: int,
) -> tuple[float, float, float, float] | None:
    x1 = float(roi.x_offset)
    y1 = float(roi.y_offset)
    x2 = x1 + float(roi.width)
    y2 = y1 + float(roi.height)
    x1 = max(0.0, min(x1, width - 1))
    y1 = max(0.0, min(y1, height - 1))
    x2 = max(0.0, min(x2, float(width)))
    y2 = max(0.0, min(y2, float(height)))
    if x2 - x1 < 1.0 or y2 - y1 < 1.0:
        return None
    return (x1, y1, x2, y2)


def _to_pose_with_meta(res: MegaResult, header) -> PoseWithMeta:
    pw = PoseWithMeta()
    pw.object_id = int(res.object_id)
    pw.class_name = res.class_name
    pw.source = 'megapose'
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
    """Rotation matrix → (x, y, z, w) quaternion (Shepperd's method)."""
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
    node = MegaPoseNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
