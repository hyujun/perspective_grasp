"""Abstract base class for grasp planners."""

from __future__ import annotations

from abc import ABC, abstractmethod
from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from typing import Callable

    from rclpy.node import Node

from perception_msgs.action import PlanGrasp


class BasePlanner(ABC):
    """All grasp planners must inherit from this class.

    Subclasses implement ``plan()`` which receives the action goal and a
    feedback callback, and returns a ``PlanGrasp.Result``.
    """

    def __init__(self, node: Node) -> None:
        """Store a reference to the owning ROS node for logging / params."""
        self._node = node

    # ------------------------------------------------------------------
    # Public API
    # ------------------------------------------------------------------

    @abstractmethod
    def plan(
        self,
        goal: PlanGrasp.Goal,
        publish_feedback: Callable[[PlanGrasp.Feedback], None],
    ) -> PlanGrasp.Result:
        """Execute the grasp planning algorithm.

        Parameters
        ----------
        goal:
            The ``PlanGrasp`` action goal containing ``target_object_id``
            and ``grasp_strategy``.
        publish_feedback:
            Callable that publishes a ``PlanGrasp.Feedback`` message back
            to the action client.

        Returns
        -------
        PlanGrasp.Result
            Must populate at minimum ``success``, ``message``, and
            ``grasp_pose``.
        """
        ...
