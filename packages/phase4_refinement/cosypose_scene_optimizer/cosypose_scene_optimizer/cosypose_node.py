"""CosyPose scene-level optimization action server.

LifecycleNode + ActionServer hybrid:

* **Subscriptions** (BEST_EFFORT, depth=1) cache the latest RGB frame,
  camera intrinsics, and YOLO detections so an ``analyze_scene`` goal
  can always solve against a recent observation.
* **Action ``analyze_scene``** runs the configured backend on the
  freshest RGB + detections inside the sync tolerance window, returns
  the optimized poses inside ``AnalyzeScene.Result``, and also
  publishes them on ``/cosypose/optimized_poses`` for any latched
  consumer (e.g. ``perception_debug_visualizer``).

Backends live under ``backends/`` (``cosypose`` / ``mock``) — selection
is driven by the ``backend`` parameter. Heavy imports are deferred to
``on_activate``.
"""

from __future__ import annotations

import importlib
import threading
from collections import deque
from typing import Any

import numpy as np
import rclpy
from cv_bridge import CvBridge
from geometry_msgs.msg import PoseStamped
from rclpy.action import ActionServer
from rclpy.lifecycle import LifecycleNode, LifecycleState, TransitionCallbackReturn
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CameraInfo, Image, RegionOfInterest

from perception_msgs.action import AnalyzeScene
from perception_msgs.msg import (
    DetectionArray,
    ObjectRelation,
    PoseWithMeta,
    PoseWithMetaArray,
)

from cosypose_scene_optimizer.backends.base_backend import (
    BaseSceneBackend,
    SceneDetection,
    SceneResult,
)


BACKEND_REGISTRY: dict[str, str] = {
    'cosypose':
        'cosypose_scene_optimizer.backends.cosypose_backend:CosyPoseBackend',
    'mock':
        'cosypose_scene_optimizer.backends.mock_backend:MockSceneBackend',
}


def _load_backend(key: str, node) -> BaseSceneBackend:
    if key not in BACKEND_REGISTRY:
        avail = ', '.join(sorted(BACKEND_REGISTRY))
        raise ValueError(f"Unknown backend '{key}'. Available: [{avail}]")
    module_path, class_name = BACKEND_REGISTRY[key].rsplit(':', 1)
    module = importlib.import_module(module_path)
    cls = getattr(module, class_name)
    if not issubclass(cls, BaseSceneBackend):
        raise TypeError(f'{cls.__name__} is not a BaseSceneBackend')
    return cls(node)


class CosyPoseNode(LifecycleNode):
    """Scene-level pose optimizer with pluggable backend."""

    def __init__(self) -> None:
        super().__init__('cosypose_optimizer')

        self._backend_type: str = (
            self.declare_parameter('backend', 'cosypose')
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
            self.declare_parameter('poses_topic', '/cosypose/optimized_poses')
            .get_parameter_value().string_value
        )
        self._action_name: str = (
            self.declare_parameter('action_name', 'analyze_scene')
            .get_parameter_value().string_value
        )
        self._frame_queue_size: int = (
            self.declare_parameter('frame_queue_size', 10)
            .get_parameter_value().integer_value
        )
        self._sync_tolerance_sec: float = (
            self.declare_parameter('sync_tolerance_sec', 0.2)
            .get_parameter_value().double_value
        )
        self.declare_parameter('min_detection_confidence', 0.0)

        self._bridge = CvBridge()
        self._backend: BaseSceneBackend | None = None
        self._K: np.ndarray | None = None

        self._state_lock = threading.Lock()
        # (stamp_ns, rgb, frame_id)
        self._image_queue: deque[tuple[int, np.ndarray, str]] = deque()
        # most recent detections seen on the topic
        self._last_detections: DetectionArray | None = None

        self._sub_image = None
        self._sub_camera_info = None
        self._sub_detections = None
        self._pub_poses = None
        self._action_server: ActionServer | None = None
        self._warned_no_camera_info = False

        self.get_logger().info(
            f"CosyPoseNode created (backend='{self._backend_type}')"
        )

    # ------------------------------------------------------------------
    # Lifecycle transitions
    # ------------------------------------------------------------------

    def on_configure(self, state: LifecycleState) -> TransitionCallbackReturn:
        self.get_logger().info('Configuring CosyPoseNode')

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
        self._pub_poses = self.create_lifecycle_publisher(
            PoseWithMetaArray, self._poses_topic, qos,
        )
        self._action_server = ActionServer(
            self, AnalyzeScene, self._action_name, self._execute_callback,
        )
        self.get_logger().info(
            f'subs: rgb={self._image_topic} info={self._camera_info_topic} '
            f'dets={self._detections_topic}  pub: {self._poses_topic}  '
            f'action: {self._action_name}'
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
        with self._state_lock:
            self._image_queue.clear()
            self._last_detections = None
        return super().on_deactivate(state)

    def on_cleanup(self, state: LifecycleState) -> TransitionCallbackReturn:
        self.get_logger().info('Cleaning up CosyPoseNode')
        if self._action_server is not None:
            self._action_server.destroy()
            self._action_server = None
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
        with self._state_lock:
            self._image_queue.clear()
            self._last_detections = None
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
        with self._state_lock:
            self._image_queue.append((stamp_ns, rgb, msg.header.frame_id))
            while len(self._image_queue) > self._frame_queue_size:
                self._image_queue.popleft()

    def _detections_callback(self, msg: DetectionArray) -> None:
        with self._state_lock:
            self._last_detections = msg

    # ------------------------------------------------------------------
    # Action callback
    # ------------------------------------------------------------------

    def _execute_callback(self, goal_handle):
        goal: AnalyzeScene.Goal = goal_handle.request
        result = AnalyzeScene.Result()
        result.optimized_poses = []
        result.relations = []

        # --- sanity ------------------------------------------------------
        if self._backend is None or not self._backend.loaded:
            goal_handle.abort()
            result.success = False
            result.message = 'backend not active (lifecycle must be ACTIVE)'
            return result
        if self._K is None:
            if not self._warned_no_camera_info:
                self.get_logger().warn('CameraInfo not yet received')
                self._warned_no_camera_info = True
            goal_handle.abort()
            result.success = False
            result.message = 'no CameraInfo received yet'
            return result

        # --- pull a paired (rgb, detections) sample ----------------------
        feedback = AnalyzeScene.Feedback()
        feedback.progress = 0.1
        feedback.current_stage = 'pair'
        goal_handle.publish_feedback(feedback)

        paired = self._latest_paired_sample()
        if paired is None:
            goal_handle.abort()
            result.success = False
            result.message = (
                f'no rgb+detections within {self._sync_tolerance_sec:.3f}s'
            )
            return result
        rgb, frame_id, header, detections = paired

        # --- filter / convert -------------------------------------------
        target_classes = list(goal.target_classes) if goal.target_classes else None
        min_conf = (
            self.get_parameter('min_detection_confidence')
            .get_parameter_value().double_value
        )
        h, w = rgb.shape[:2]
        scene_dets: list[SceneDetection] = []
        for det in detections.detections:
            if det.confidence < min_conf:
                continue
            box = _roi_to_xyxy(det.bbox, w, h)
            if box is None:
                continue
            scene_dets.append(SceneDetection(
                object_id=int(det.id),
                class_name=det.class_name,
                bbox_xyxy=box,
                confidence=float(det.confidence),
            ))
        if not scene_dets:
            goal_handle.succeed()
            result.success = True
            result.message = 'no valid detections to optimize'
            return result

        # --- coarse + refine inside the backend --------------------------
        feedback.progress = 0.4
        feedback.current_stage = 'coarse'
        goal_handle.publish_feedback(feedback)
        try:
            results = self._backend.optimize(
                rgb, self._K, scene_dets, target_classes,
            )
        except Exception as e:  # noqa: BLE001
            self.get_logger().error(f'scene optimization failed: {e}')
            goal_handle.abort()
            result.success = False
            result.message = f'backend error: {e}'
            return result

        feedback.progress = 0.9
        feedback.current_stage = 'publish'
        goal_handle.publish_feedback(feedback)

        # --- build + publish outputs -------------------------------------
        out = PoseWithMetaArray()
        out.header = header
        if not out.header.frame_id:
            out.header.frame_id = frame_id
        for res in results:
            out.poses.append(_to_pose_with_meta(res, out.header))

        if self._pub_poses is not None:
            self._pub_poses.publish(out)

        result.optimized_poses = list(out.poses)
        result.relations = []  # relation inference is a TODO
        result.success = True
        result.message = f'optimized {len(out.poses)} / {len(scene_dets)} detections'
        goal_handle.succeed()
        return result

    # ------------------------------------------------------------------
    # Helpers
    # ------------------------------------------------------------------

    def _latest_paired_sample(
        self,
    ) -> tuple[np.ndarray, str, Any, DetectionArray] | None:
        """Return the freshest (rgb, frame_id, header, detections) within tol."""
        tol_ns = int(self._sync_tolerance_sec * 1e9)
        with self._state_lock:
            if not self._image_queue or self._last_detections is None:
                return None
            det = self._last_detections
            target_ns = _stamp_to_ns(det.header.stamp)
            best: tuple[int, np.ndarray, str] | None = None
            best_diff = tol_ns + 1
            for stamp_ns, rgb, frame in self._image_queue:
                diff = abs(stamp_ns - target_ns)
                if diff < best_diff:
                    best_diff = diff
                    best = (stamp_ns, rgb, frame)
            if best is None or best_diff > tol_ns:
                return None
            return best[1], best[2], det.header, det


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


def _to_pose_with_meta(res: SceneResult, header) -> PoseWithMeta:
    pw = PoseWithMeta()
    pw.object_id = int(res.object_id)
    pw.class_name = res.class_name
    pw.source = 'cosypose'
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
    node = CosyPoseNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
