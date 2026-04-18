"""Abstract base class for robot-specific gripper adapters."""

from __future__ import annotations

from abc import ABC, abstractmethod
from typing import TYPE_CHECKING

import numpy as np

if TYPE_CHECKING:
    from rclpy.node import Node


class BaseGripper(ABC):
    """Robot-specific end-effector surface used by grasp planners.

    The planner reaches into the gripper for:
      * kinematic limits (min/max opening width)
      * frame conventions (approach/closing axes in gripper frame)
      * joint configuration for a given grasp width + strategy
      * collision geometry used during clearance checks

    Concrete subclasses (e.g. ``Hand10DoF``, ``ParallelJaw``) are
    registered in ``grasp_planner_node.GRIPPER_REGISTRY``.
    """

    def __init__(self, node: Node) -> None:
        self._node = node

    # ------------------------------------------------------------------
    # Kinematic limits
    # ------------------------------------------------------------------

    @property
    @abstractmethod
    def width_min(self) -> float:
        """Minimum reachable opening width in meters (fully closed)."""

    @property
    @abstractmethod
    def width_max(self) -> float:
        """Maximum reachable opening width in meters (fully open)."""

    # ------------------------------------------------------------------
    # Frame conventions (unit vectors, gripper frame)
    # ------------------------------------------------------------------

    @property
    @abstractmethod
    def approach_axis(self) -> np.ndarray:
        """Direction from palm outward toward the object. Shape (3,)."""

    @property
    @abstractmethod
    def closing_axis(self) -> np.ndarray:
        """Direction along which the fingers close. Shape (3,)."""

    # ------------------------------------------------------------------
    # Behavioural surface
    # ------------------------------------------------------------------

    @abstractmethod
    def joint_config(self, width: float, strategy: str) -> list[float]:
        """Return joint positions for the given grasp width + strategy.

        Parameters
        ----------
        width:
            Target opening width at the contact points (meters).
        strategy:
            ``'power'`` / ``'precision'`` / ``'pinch'`` — higher-level
            grasp type the planner decided on.
        """

    @abstractmethod
    def collision_points(
        self, grasp_pose: np.ndarray, width: float,
    ) -> np.ndarray:
        """Points (Nx3) approximating the gripper volume at a candidate pose.

        Used by the planner's feasibility stage: any scene point falling
        inside this set means the gripper would collide.

        ``grasp_pose`` is a 4x4 homogeneous transform placing the gripper
        frame in the same frame as the scene cloud.
        """
