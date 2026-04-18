"""10-DoF anthropomorphic hand adapter.

TODO(hand-mapping): The actual joint ordering and URDF for the production
hand is not yet wired in. This file provides the correct ``BaseGripper``
surface so the planner can run end-to-end, but ``joint_config`` returns
placeholder values that must be remapped once the hand's URDF joint order
and per-strategy preshape are confirmed.
"""

from __future__ import annotations

from typing import TYPE_CHECKING

import numpy as np

from .base_gripper import BaseGripper

if TYPE_CHECKING:
    from rclpy.node import Node


_JOINT_COUNT = 10

# Placeholder preshapes — one entry per joint, length must equal _JOINT_COUNT.
# TODO(hand-mapping): replace with real joint ordering from the 10-DoF hand's
# URDF and tune per strategy. Values are in radians.
_PRESHAPES: dict[str, list[float]] = {
    'power':     [0.0] * _JOINT_COUNT,
    'precision': [0.0] * _JOINT_COUNT,
    'pinch':     [0.0] * _JOINT_COUNT,
}


class Hand10DoF(BaseGripper):
    """Adapter for the 10-DoF hand mounted on the UR5e.

    Exposes kinematic limits and a strategy-based joint preshape. Collision
    geometry is approximated as a palm box + two finger "rails" parameterised
    by the current opening width.
    """

    def __init__(self, node: Node) -> None:
        super().__init__(node)

        self._width_min: float = (
            node.declare_parameter('gripper.width_min', 0.0)
            .get_parameter_value().double_value
        )
        self._width_max: float = (
            node.declare_parameter('gripper.width_max', 0.12)
            .get_parameter_value().double_value
        )
        self._palm_depth: float = (
            node.declare_parameter('gripper.palm_depth', 0.05)
            .get_parameter_value().double_value
        )
        self._palm_height: float = (
            node.declare_parameter('gripper.palm_height', 0.06)
            .get_parameter_value().double_value
        )
        self._finger_length: float = (
            node.declare_parameter('gripper.finger_length', 0.09)
            .get_parameter_value().double_value
        )
        self._finger_thickness: float = (
            node.declare_parameter('gripper.finger_thickness', 0.02)
            .get_parameter_value().double_value
        )

        node.get_logger().info(
            f'Hand10DoF gripper  width=[{self._width_min:.3f}, '
            f'{self._width_max:.3f}] m, palm_depth={self._palm_depth:.3f}, '
            f'finger_length={self._finger_length:.3f}'
        )

    # ------------------------------------------------------------------
    # Kinematic limits
    # ------------------------------------------------------------------

    @property
    def width_min(self) -> float:
        return self._width_min

    @property
    def width_max(self) -> float:
        return self._width_max

    # ------------------------------------------------------------------
    # Frame conventions
    # ------------------------------------------------------------------

    @property
    def approach_axis(self) -> np.ndarray:
        # Gripper approaches along +Z of its own frame.
        return np.array([0.0, 0.0, 1.0])

    @property
    def closing_axis(self) -> np.ndarray:
        # Fingers close along ±Y of the gripper frame.
        return np.array([0.0, 1.0, 0.0])

    # ------------------------------------------------------------------
    # Behavioural surface
    # ------------------------------------------------------------------

    def joint_config(self, width: float, strategy: str) -> list[float]:
        if strategy not in _PRESHAPES:
            self._node.get_logger().warn(
                f"Unknown grasp_strategy '{strategy}', defaulting to 'power'"
            )
            strategy = 'power'

        # TODO(hand-mapping): width → joint interpolation. For now we return
        # the preshape verbatim; once the URDF joint ordering is known we
        # should scale the abduction/flexion joints by
        #     alpha = (width - width_min) / (width_max - width_min)
        # and apply per-joint min/max limits from the URDF.
        return list(_PRESHAPES[strategy])

    def collision_points(
        self, grasp_pose: np.ndarray, width: float,
    ) -> np.ndarray:
        """Approximate the hand volume with a palm box and two finger rails.

        All geometry is sampled in the gripper frame (palm centred at origin,
        fingers extending toward +Z), then transformed by ``grasp_pose``.
        """
        half_w = 0.5 * max(width + self._finger_thickness, self._finger_thickness)

        # --- Palm box: behind the fingertips along -Z -----------------
        nx, ny, nz = 4, 4, 3
        xs = np.linspace(-0.5 * self._palm_height, 0.5 * self._palm_height, nx)
        ys = np.linspace(-0.5 * self._palm_height, 0.5 * self._palm_height, ny)
        zs = np.linspace(-self._palm_depth, 0.0, nz)
        palm = np.stack(np.meshgrid(xs, ys, zs, indexing='ij'), axis=-1).reshape(-1, 3)

        # --- Two finger rails: extending +Z from palm front -----------
        fz = np.linspace(0.0, self._finger_length, 6)
        fx = np.linspace(-0.5 * self._finger_thickness,
                         0.5 * self._finger_thickness, 2)
        rail_x, rail_z = np.meshgrid(fx, fz, indexing='ij')
        rail_pts = np.stack(
            [rail_x.ravel(),
             np.full(rail_x.size, 0.0),
             rail_z.ravel()], axis=-1,
        )
        left = rail_pts + np.array([0.0, -half_w, 0.0])
        right = rail_pts + np.array([0.0, +half_w, 0.0])

        local = np.vstack([palm, left, right])  # (M, 3)

        # Transform into the scene frame.
        R = grasp_pose[:3, :3]
        t = grasp_pose[:3, 3]
        return local @ R.T + t
