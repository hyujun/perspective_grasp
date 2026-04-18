"""SAM2 instance segmentation LifecycleNode.

Pairs YOLO detections with their source image by timestamp, prompts the
configured segmentation backend with each box, and republishes the
resulting masks as a ``SegmentationArray``.

Concrete inference lives in ``backends/`` so the node stays
framework-agnostic (real SAM2 for the Docker service, ``mock`` for
CPU-only smoke tests).
"""

from __future__ import annotations

import importlib
import threading
from collections import deque
from typing import TYPE_CHECKING

import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.lifecycle import LifecycleNode, LifecycleState, TransitionCallbackReturn
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Image, RegionOfInterest

from perception_msgs.msg import (
    Detection,
    DetectionArray,
    Segmentation,
    SegmentationArray,
)

from sam2_instance_segmentor.backends.base_backend import BaseBackend

if TYPE_CHECKING:
    pass


# ── Backend registry ─────────────────────────────────────────────────
BACKEND_REGISTRY: dict[str, str] = {
    'sam2': 'sam2_instance_segmentor.backends.sam2_backend:Sam2Backend',
    'mock': 'sam2_instance_segmentor.backends.mock_backend:MockBackend',
}


def _load_backend(key: str, node) -> BaseBackend:
    if key not in BACKEND_REGISTRY:
        avail = ', '.join(sorted(BACKEND_REGISTRY))
        raise ValueError(f"Unknown backend '{key}'. Available: [{avail}]")
    module_path, class_name = BACKEND_REGISTRY[key].rsplit(':', 1)
    module = importlib.import_module(module_path)
    cls = getattr(module, class_name)
    if not issubclass(cls, BaseBackend):
        raise TypeError(f'{cls.__name__} is not a BaseBackend')
    return cls(node)


class Sam2SegmentorNode(LifecycleNode):
    """LifecycleNode wrapping a box-prompted segmentation backend."""

    def __init__(self) -> None:
        super().__init__('sam2_segmentor')

        self._backend_type: str = (
            self.declare_parameter('backend', 'sam2')
            .get_parameter_value().string_value
        )
        self._image_topic: str = (
            self.declare_parameter('image_topic', '/camera/color/image_raw')
            .get_parameter_value().string_value
        )
        self._detections_topic: str = (
            self.declare_parameter('detections_topic', '/yolo/detections')
            .get_parameter_value().string_value
        )
        self._masks_topic: str = (
            self.declare_parameter('masks_topic', '/sam2/masks')
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
        # Declared up front; value is re-read on every detection callback so
        # runtime `ros2 param set` takes effect without a lifecycle restart.
        self.declare_parameter('min_detection_confidence', 0.0)

        self._bridge = CvBridge()
        self._backend: BaseBackend | None = None
        self._image_lock = threading.Lock()
        self._image_queue: deque[tuple[int, np.ndarray, str]] = deque()

        self._sub_image = None
        self._sub_detections = None
        self._pub_masks = None

        self.get_logger().info(
            f"Sam2SegmentorNode created (backend='{self._backend_type}')"
        )

    # ------------------------------------------------------------------
    # Lifecycle transitions
    # ------------------------------------------------------------------

    def on_configure(self, state: LifecycleState) -> TransitionCallbackReturn:
        self.get_logger().info('Configuring Sam2SegmentorNode')

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
        self._sub_detections = self.create_subscription(
            DetectionArray, self._detections_topic,
            self._detections_callback, qos,
        )
        self._pub_masks = self.create_publisher(
            SegmentationArray, self._masks_topic, qos,
        )
        self.get_logger().info(
            f'subs: {self._image_topic}, {self._detections_topic}  '
            f'pub: {self._masks_topic}'
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
        self.get_logger().info('Cleaning up Sam2SegmentorNode')
        if self._backend is not None:
            self._backend.unload()
            self._backend = None
        for sub in (self._sub_image, self._sub_detections):
            if sub is not None:
                self.destroy_subscription(sub)
        if self._pub_masks is not None:
            self.destroy_publisher(self._pub_masks)
        self._sub_image = None
        self._sub_detections = None
        self._pub_masks = None
        with self._image_lock:
            self._image_queue.clear()
        return TransitionCallbackReturn.SUCCESS

    # ------------------------------------------------------------------
    # Subscription callbacks
    # ------------------------------------------------------------------

    def _image_callback(self, msg: Image) -> None:
        try:
            img = self._bridge.imgmsg_to_cv2(msg, desired_encoding='rgb8')
        except Exception as e:  # noqa: BLE001
            self.get_logger().warn(f'cv_bridge failed: {e}')
            return
        stamp_ns = _stamp_to_ns(msg.header.stamp)
        with self._image_lock:
            self._image_queue.append((stamp_ns, img, msg.header.frame_id))
            while len(self._image_queue) > self._image_queue_size:
                self._image_queue.popleft()

    def _detections_callback(self, msg: DetectionArray) -> None:
        if self._backend is None or not self._backend.loaded:
            return
        if not msg.detections:
            return

        target_ns = _stamp_to_ns(msg.header.stamp)
        paired = self._pair_image(target_ns)
        if paired is None:
            self.get_logger().debug(
                f'no image within {self._sync_tolerance_sec:.3f}s of '
                f'detections at t={target_ns}'
            )
            return
        img, frame_id = paired

        min_conf = (
            self.get_parameter('min_detection_confidence')
            .get_parameter_value().double_value
        )
        kept: list[Detection] = []
        boxes: list[list[float]] = []
        h, w = img.shape[:2]
        for det in msg.detections:
            if det.confidence < min_conf:
                continue
            box = _roi_to_xyxy(det.bbox, w, h)
            if box is None:
                continue
            kept.append(det)
            boxes.append(box)

        if not kept:
            return

        try:
            results = self._backend.segment(img, np.asarray(boxes))
        except Exception as e:  # noqa: BLE001
            self.get_logger().error(f'segmentation failed: {e}')
            return

        out = SegmentationArray()
        out.header = msg.header
        if not out.header.frame_id:
            out.header.frame_id = frame_id

        for det, res in zip(kept, results, strict=True):
            seg = Segmentation()
            seg.id = det.id
            seg.bbox = det.bbox
            seg.confidence = float(res.score)
            seg.mask = self._bridge.cv2_to_imgmsg(
                res.mask.astype(np.uint8) * 255, encoding='mono8',
            )
            seg.mask.header = out.header
            out.segmentations.append(seg)

        assert self._pub_masks is not None
        self._pub_masks.publish(out)

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
            for stamp_ns, img, frame in self._image_queue:
                diff = abs(stamp_ns - target_ns)
                if diff < best_diff:
                    best_diff = diff
                    best = (stamp_ns, img, frame)
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
) -> list[float] | None:
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
    return [x1, y1, x2, y2]


def main(args=None):
    rclpy.init(args=args)
    node = Sam2SegmentorNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
