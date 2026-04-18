"""Grasp pose planning action server with pluggable planner + gripper.

The node owns the ROS interfaces (subscriptions, action server) and caches
the latest filtered poses and merged scene cloud. Concrete planners and
gripper adapters are selected at runtime via registries so that the
planning algorithm stays robot-agnostic and the robot-specific parts live
in the gripper adapter.
"""

from __future__ import annotations

import importlib
import threading
from typing import TYPE_CHECKING

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.action import ActionServer
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2

from perception_msgs.action import PlanGrasp
from perception_msgs.msg import PoseWithMetaArray

from grasp_pose_planner.grippers.base_gripper import BaseGripper
from grasp_pose_planner.planners.base_planner import BasePlanner

if TYPE_CHECKING:
    pass


# ── Registries ────────────────────────────────────────────────────────
# Planner: the grasp-generation algorithm (robot-agnostic).
PLANNER_REGISTRY: dict[str, str] = {
    'antipodal': 'grasp_pose_planner.planners.antipodal_planner:AntipodalPlanner',
}

# Gripper: robot-specific end-effector adapter.
GRIPPER_REGISTRY: dict[str, str] = {
    'hand_10dof': 'grasp_pose_planner.grippers.hand_10dof:Hand10DoF',
}


def _load(registry: dict[str, str], key: str, base_cls, *args):
    if key not in registry:
        available = ', '.join(sorted(registry))
        raise ValueError(f"Unknown key '{key}'. Available: [{available}]")
    module_path, class_name = registry[key].rsplit(':', 1)
    module = importlib.import_module(module_path)
    cls = getattr(module, class_name)
    if not issubclass(cls, base_cls):
        raise TypeError(f'{cls.__name__} is not a subclass of {base_cls.__name__}')
    return cls(*args)


# ── ROS 2 Node ────────────────────────────────────────────────────────

class GraspPlannerNode(Node):
    """Action server + data cache for grasp pose planning.

    ROS interfaces:
      * Sub  ``/pose_filter/filtered_poses`` (``PoseWithMetaArray``)
      * Sub  ``/merged/points`` (``PointCloud2``)
      * Act  ``/plan_grasp`` (``PlanGrasp``)

    Concrete behaviour is selected by two ROS parameters:
      * ``planner_type`` — key in ``PLANNER_REGISTRY``
      * ``gripper_type`` — key in ``GRIPPER_REGISTRY``
    """

    def __init__(self) -> None:
        super().__init__('grasp_planner')

        planner_type: str = (
            self.declare_parameter('planner_type', 'antipodal')
            .get_parameter_value().string_value
        )
        gripper_type: str = (
            self.declare_parameter('gripper_type', 'hand_10dof')
            .get_parameter_value().string_value
        )
        self._max_pose_age: float = (
            self.declare_parameter('max_pose_age_sec', 1.0)
            .get_parameter_value().double_value
        )
        self._max_cloud_age: float = (
            self.declare_parameter('max_cloud_age_sec', 1.0)
            .get_parameter_value().double_value
        )

        self._pose_topic: str = (
            self.declare_parameter('pose_topic', '/pose_filter/filtered_poses')
            .get_parameter_value().string_value
        )
        self._cloud_topic: str = (
            self.declare_parameter('cloud_topic', '/merged/points')
            .get_parameter_value().string_value
        )

        # ── Load adapter + planner ──────────────────────────────────
        self.get_logger().info(
            f'Loading gripper={gripper_type}, planner={planner_type}'
        )
        self._gripper: BaseGripper = _load(
            GRIPPER_REGISTRY, gripper_type, BaseGripper, self,
        )
        self._planner: BasePlanner = _load(
            PLANNER_REGISTRY, planner_type, BasePlanner, self, self._gripper,
        )

        # ── Data caches (guarded by lock) ───────────────────────────
        self._cache_lock = threading.Lock()
        self._latest_poses: dict[int, PoseStamped] = {}
        self._latest_poses_stamp: rclpy.time.Time | None = None
        self._latest_cloud: np.ndarray | None = None
        self._latest_cloud_frame: str = ''
        self._latest_cloud_stamp: rclpy.time.Time | None = None

        # ── Subscriptions (control-path QoS) ───────────────────────
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self._sub_poses = self.create_subscription(
            PoseWithMetaArray, self._pose_topic, self._on_poses, qos,
        )
        self._sub_cloud = self.create_subscription(
            PointCloud2, self._cloud_topic, self._on_cloud, qos,
        )

        # ── Action server ───────────────────────────────────────────
        self._action_server = ActionServer(
            self, PlanGrasp, 'plan_grasp', self._execute_callback,
        )
        self.get_logger().info(
            'GraspPlannerNode ready  (action: /plan_grasp, '
            f'pose_topic={self._pose_topic}, cloud_topic={self._cloud_topic})'
        )

    # ------------------------------------------------------------------
    # Subscription callbacks
    # ------------------------------------------------------------------

    def _on_poses(self, msg: PoseWithMetaArray) -> None:
        new_poses: dict[int, PoseStamped] = {}
        for entry in msg.poses:
            new_poses[int(entry.object_id)] = entry.pose
        with self._cache_lock:
            self._latest_poses = new_poses
            self._latest_poses_stamp = rclpy.time.Time.from_msg(msg.header.stamp)

    def _on_cloud(self, msg: PointCloud2) -> None:
        try:
            pts = point_cloud2.read_points_numpy(
                msg, field_names=('x', 'y', 'z'), skip_nans=True,
            )
        except Exception as e:  # noqa: BLE001
            self.get_logger().warn(f'PointCloud2 decode failed: {e}')
            return
        arr = np.ascontiguousarray(pts, dtype=np.float64)
        with self._cache_lock:
            self._latest_cloud = arr
            self._latest_cloud_frame = msg.header.frame_id
            self._latest_cloud_stamp = rclpy.time.Time.from_msg(msg.header.stamp)

    # ------------------------------------------------------------------
    # Planner-facing helpers (thread-safe)
    # ------------------------------------------------------------------

    def get_latest_pose(self, object_id: int) -> PoseStamped | None:
        with self._cache_lock:
            stamp = self._latest_poses_stamp
            pose = self._latest_poses.get(int(object_id))
        if pose is None:
            return None
        if not self._fresh(stamp, self._max_pose_age):
            return None
        return pose

    def get_scene_cloud(self) -> tuple[np.ndarray, str] | None:
        with self._cache_lock:
            cloud = self._latest_cloud
            frame = self._latest_cloud_frame
            stamp = self._latest_cloud_stamp
        if cloud is None:
            return None
        if not self._fresh(stamp, self._max_cloud_age):
            return None
        return cloud, frame

    def _fresh(self, stamp: rclpy.time.Time | None, max_age: float) -> bool:
        if stamp is None or max_age <= 0.0:
            return stamp is not None
        age_ns = (self.get_clock().now() - stamp).nanoseconds
        return age_ns < max_age * 1e9

    # ------------------------------------------------------------------
    # Action handler
    # ------------------------------------------------------------------

    def _execute_callback(self, goal_handle):
        self.get_logger().info(
            f'PlanGrasp goal: object_id={goal_handle.request.target_object_id}, '
            f"strategy='{goal_handle.request.grasp_strategy}'"
        )
        try:
            result = self._planner.plan(
                goal_handle.request, goal_handle.publish_feedback,
            )
        except Exception as e:  # noqa: BLE001
            self.get_logger().error(f'Planner raised: {e}')
            result = PlanGrasp.Result()
            result.success = False
            result.message = f'planner exception: {e}'

        goal_handle.succeed()  # action completes; client checks result.success
        return result


def main(args=None):
    rclpy.init(args=args)
    node = GraspPlannerNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
