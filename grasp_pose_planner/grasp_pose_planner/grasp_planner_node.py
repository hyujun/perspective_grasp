"""Grasp pose planning action server with pluggable planner strategy."""

from __future__ import annotations

import rclpy
from rclpy.action import ActionServer
from rclpy.node import Node

from perception_msgs.action import PlanGrasp

from grasp_pose_planner.planners.base_planner import BasePlanner

# ── Planner registry ─────────────────────────────────────────────────
# Add new planners here:  'key': 'module_path:ClassName'
PLANNER_REGISTRY: dict[str, str] = {
    'antipodal': 'grasp_pose_planner.planners.antipodal_planner:AntipodalPlanner',
}


def _load_planner(key: str, node: Node) -> BasePlanner:
    """Instantiate a planner by registry key (lazy import)."""
    if key not in PLANNER_REGISTRY:
        available = ', '.join(sorted(PLANNER_REGISTRY))
        raise ValueError(
            f"Unknown planner_type '{key}'. Available: [{available}]"
        )
    module_path, class_name = PLANNER_REGISTRY[key].rsplit(':', 1)

    import importlib
    module = importlib.import_module(module_path)
    cls = getattr(module, class_name)

    if not issubclass(cls, BasePlanner):
        raise TypeError(f'{cls.__name__} is not a subclass of BasePlanner')

    return cls(node)


# ── ROS 2 Node ────────────────────────────────────────────────────────

class GraspPlannerNode(Node):
    """Action server for grasp pose planning.

    The concrete planning algorithm is selected via the ``planner_type``
    ROS parameter.  See ``PLANNER_REGISTRY`` for available options.
    """

    def __init__(self) -> None:
        super().__init__('grasp_planner')

        planner_type: str = (
            self.declare_parameter('planner_type', 'antipodal')
            .get_parameter_value().string_value
        )
        self.get_logger().info(f'Loading planner: {planner_type}')
        self._planner = _load_planner(planner_type, self)

        self._action_server = ActionServer(
            self, PlanGrasp, 'plan_grasp', self._execute_callback,
        )
        self.get_logger().info('GraspPlannerNode ready  (action: /plan_grasp)')

    def _execute_callback(self, goal_handle):
        self.get_logger().info(
            f'PlanGrasp goal: object_id={goal_handle.request.target_object_id}, '
            f'strategy={goal_handle.request.grasp_strategy}'
        )

        result = self._planner.plan(
            goal_handle.request, goal_handle.publish_feedback,
        )

        if result.success:
            goal_handle.succeed()
        else:
            goal_handle.succeed()  # action still "succeeds"; check result.success

        return result


def main(args=None):
    rclpy.init(args=args)
    node = GraspPlannerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
