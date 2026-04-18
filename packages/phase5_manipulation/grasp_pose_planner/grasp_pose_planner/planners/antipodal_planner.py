"""Antipodal heuristic grasp planner (example implementation)."""

from __future__ import annotations

from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from typing import Callable

    from rclpy.node import Node

from geometry_msgs.msg import PoseStamped
from perception_msgs.action import PlanGrasp

from .base_planner import BasePlanner


class AntipodalPlanner(BasePlanner):
    """Heuristic antipodal grasp planner.

    Generates candidate grasps using an antipodal heuristic, scores them,
    and returns the best candidate.  This is a skeleton — replace the body
    of ``plan()`` with your actual algorithm.
    """

    def __init__(self, node: Node) -> None:
        super().__init__(node)
        # Read planner-specific parameters from the node
        self._num_candidates: int = (
            node.declare_parameter('num_grasp_candidates', 100)
            .get_parameter_value().integer_value
        )
        self._antipodal_threshold: float = (
            node.declare_parameter('antipodal_threshold', 0.7)
            .get_parameter_value().double_value
        )
        self._collision_check: bool = (
            node.declare_parameter('collision_check', True)
            .get_parameter_value().bool_value
        )
        self._approach_distance: float = (
            node.declare_parameter('approach_distance', 0.10)
            .get_parameter_value().double_value
        )
        self._gripper_width_max: float = (
            node.declare_parameter('gripper_width_max', 0.085)
            .get_parameter_value().double_value
        )
        node.get_logger().info(
            f'AntipodalPlanner initialised  '
            f'(candidates={self._num_candidates}, '
            f'threshold={self._antipodal_threshold})'
        )

    # ------------------------------------------------------------------
    # BasePlanner interface
    # ------------------------------------------------------------------

    def plan(
        self,
        goal: PlanGrasp.Goal,
        publish_feedback: Callable[[PlanGrasp.Feedback], None],
    ) -> PlanGrasp.Result:
        logger = self._node.get_logger()
        logger.info(
            f'AntipodalPlanner: planning for object {goal.target_object_id} '
            f'strategy={goal.grasp_strategy}'
        )

        # --- Stage 1: candidate generation --------------------------------
        feedback = PlanGrasp.Feedback()
        feedback.current_stage = 'generation'
        feedback.num_candidates = 0
        feedback.best_score_so_far = 0.0
        publish_feedback(feedback)

        # TODO: Replace with actual candidate generation logic
        #   e.g. sample surface normals from the object point cloud,
        #   form antipodal pairs, and compute grasp frames.

        # --- Stage 2: feasibility check -----------------------------------
        feedback.current_stage = 'feasibility'
        publish_feedback(feedback)

        # TODO: collision checking, kinematic reachability, etc.

        # --- Stage 3: selection -------------------------------------------
        feedback.current_stage = 'selection'
        publish_feedback(feedback)

        # TODO: rank candidates and pick the best one.

        # --- Build result (stub values) -----------------------------------
        result = PlanGrasp.Result()
        result.grasp_pose = PoseStamped()
        result.finger_joint_config = []
        result.approach_waypoints = []
        result.grasp_quality_score = 0.0
        result.success = False
        result.message = (
            'AntipodalPlanner: algorithm not yet implemented — '
            'override plan() with your logic'
        )
        return result
