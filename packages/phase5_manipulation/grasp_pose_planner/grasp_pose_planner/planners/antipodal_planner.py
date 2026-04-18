"""Antipodal grasp planner (robot-agnostic geometry)."""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import TYPE_CHECKING

import numpy as np
from perception_msgs.action import PlanGrasp

from grasp_pose_planner.utils.geometry import (
    build_pose_stamped,
    compute_normals,
    frame_from_axes,
)

from .base_planner import BasePlanner

if TYPE_CHECKING:
    from typing import Callable

    from rclpy.node import Node

    from grasp_pose_planner.grippers.base_gripper import BaseGripper


# Target grasp widths (fraction of width_max) per strategy — used for scoring.
_STRATEGY_WIDTH_FRACTION: dict[str, float] = {
    'pinch':     0.25,
    'precision': 0.50,
    'power':     0.80,
}


@dataclass
class _Candidate:
    pi: np.ndarray
    pj: np.ndarray
    ni: np.ndarray
    nj: np.ndarray
    width: float
    antipodal: float
    transform: np.ndarray | None = None
    clearance: float = 0.0
    strategy_fit: float = 0.0
    final_score: float = 0.0

    @property
    def center(self) -> np.ndarray:
        return 0.5 * (self.pi + self.pj)

    @property
    def closing_axis(self) -> np.ndarray:
        d = self.pj - self.pi
        return d / (np.linalg.norm(d) + 1e-12)


class AntipodalPlanner(BasePlanner):
    """Antipodal grasp generation from a point-cloud crop of the target.

    Pipeline:
      1. **generation** — sample points, compute normals via k-NN PCA,
         form antipodal pairs whose distance sits inside the gripper's
         width range and whose normals oppose along the pair axis.
      2. **feasibility** — lift each pair to a full 6-DoF grasp frame
         (two approach directions tried), then reject poses whose gripper
         collision volume intersects the scene cloud.
      3. **selection** — weighted score of antipodal alignment, clearance,
         and strategy fit; best candidate wins.

    The planner is robot-agnostic; it only talks to the gripper adapter
    for width limits and collision geometry. Joint configuration is
    deferred to ``BaseGripper.joint_config``.
    """

    def __init__(self, node: Node, gripper: BaseGripper) -> None:
        super().__init__(node, gripper)

        self._num_samples: int = (
            node.declare_parameter('num_grasp_candidates', 100)
            .get_parameter_value().integer_value
        )
        self._antipodal_threshold: float = (
            node.declare_parameter('antipodal_threshold', 0.85)
            .get_parameter_value().double_value
        )
        self._approach_distance: float = (
            node.declare_parameter('approach_distance', 0.10)
            .get_parameter_value().double_value
        )
        self._crop_radius: float = (
            node.declare_parameter('object_crop_radius', 0.20)
            .get_parameter_value().double_value
        )
        self._min_object_points: int = (
            node.declare_parameter('min_object_points', 80)
            .get_parameter_value().integer_value
        )
        self._max_cloud_points: int = (
            node.declare_parameter('max_cloud_points', 2000)
            .get_parameter_value().integer_value
        )
        self._normals_knn: int = (
            node.declare_parameter('normals_knn', 12)
            .get_parameter_value().integer_value
        )
        self._top_k_feasibility: int = (
            node.declare_parameter('top_k_feasibility', 40)
            .get_parameter_value().integer_value
        )
        self._clearance_radius: float = (
            node.declare_parameter('clearance_radius', 0.008)
            .get_parameter_value().double_value
        )
        self._min_clearance: float = (
            node.declare_parameter('min_clearance_fraction', 0.85)
            .get_parameter_value().double_value
        )
        self._w_antipodal: float = (
            node.declare_parameter('score_weight_antipodal', 0.5)
            .get_parameter_value().double_value
        )
        self._w_clearance: float = (
            node.declare_parameter('score_weight_clearance', 0.3)
            .get_parameter_value().double_value
        )
        self._w_strategy: float = (
            node.declare_parameter('score_weight_strategy', 0.2)
            .get_parameter_value().double_value
        )

        self._rng = np.random.default_rng(
            node.declare_parameter('rng_seed', 0)
            .get_parameter_value().integer_value or None
        )

        node.get_logger().info(
            f'AntipodalPlanner initialised  '
            f'(samples={self._num_samples}, '
            f'antipodal_thr={self._antipodal_threshold}, '
            f'crop_r={self._crop_radius} m)'
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
            f'AntipodalPlanner: target={goal.target_object_id} '
            f"strategy='{goal.grasp_strategy}'"
        )

        # --- Resolve inputs --------------------------------------------
        target = self._node.get_latest_pose(goal.target_object_id)
        if target is None:
            return _fail(
                f'no filtered pose cached for object_id={goal.target_object_id}'
            )

        cloud_info = self._node.get_scene_cloud()
        if cloud_info is None:
            return _fail('no scene cloud available from /merged/points')
        cloud, cloud_frame = cloud_info

        if cloud_frame and cloud_frame != target.header.frame_id:
            return _fail(
                f'frame mismatch (pose={target.header.frame_id}, '
                f'cloud={cloud_frame}) — TF transform not implemented'
            )

        center = np.array([
            target.pose.position.x,
            target.pose.position.y,
            target.pose.position.z,
        ])
        cropped = _crop_sphere(cloud, center, self._crop_radius)
        if cropped.shape[0] < self._min_object_points:
            return _fail(
                f'only {cropped.shape[0]} points within '
                f'{self._crop_radius:.2f} m of object (need '
                f'>= {self._min_object_points})'
            )
        cropped = _random_downsample(cropped, self._max_cloud_points, self._rng)

        # --- Stage 1: candidate generation -----------------------------
        fb = PlanGrasp.Feedback()
        fb.current_stage = 'generation'
        fb.num_candidates = 0
        fb.best_score_so_far = 0.0
        publish_feedback(fb)

        normals = compute_normals(cropped, k=self._normals_knn)
        candidates = self._generate(cropped, normals)
        fb.num_candidates = len(candidates)
        publish_feedback(fb)
        if not candidates:
            return _fail(
                f'no antipodal pairs passed threshold '
                f'({self._antipodal_threshold:.2f}) among '
                f'{cropped.shape[0]} points'
            )

        candidates.sort(key=lambda c: c.antipodal, reverse=True)
        candidates = candidates[: self._top_k_feasibility]

        # --- Stage 2: feasibility --------------------------------------
        fb.current_stage = 'feasibility'
        publish_feedback(fb)

        feasible = self._check_feasibility(candidates, cloud)
        if not feasible:
            return _fail(
                f'all {len(candidates)} top candidates failed clearance '
                f'(min_fraction={self._min_clearance:.2f})'
            )

        # --- Stage 3: selection ----------------------------------------
        fb.current_stage = 'selection'
        publish_feedback(fb)

        best = self._select(feasible, goal.grasp_strategy)
        fb.best_score_so_far = float(best.final_score)
        publish_feedback(fb)

        # --- Build result ----------------------------------------------
        stamp = self._node.get_clock().now().to_msg()
        frame = target.header.frame_id or cloud_frame

        pre_T = best.transform.copy()
        pre_T[:3, 3] -= best.transform[:3, 2] * self._approach_distance

        result = PlanGrasp.Result()
        result.grasp_pose = build_pose_stamped(best.transform, frame, stamp)
        result.approach_waypoints = [build_pose_stamped(pre_T, frame, stamp)]
        result.finger_joint_config = self._gripper.joint_config(
            best.width, goal.grasp_strategy,
        )
        result.grasp_quality_score = float(best.final_score)
        result.success = True
        result.message = (
            f'ok: {len(feasible)}/{len(candidates)} feasible; '
            f'width={best.width * 1000:.1f} mm, '
            f'antipodal={best.antipodal:.2f}, '
            f'clearance={best.clearance:.2f}'
        )
        return result

    # ------------------------------------------------------------------
    # Internal stages
    # ------------------------------------------------------------------

    def _generate(
        self, points: np.ndarray, normals: np.ndarray,
    ) -> list[_Candidate]:
        n = points.shape[0]
        sample_n = min(self._num_samples, n)
        sample_idx = self._rng.choice(n, size=sample_n, replace=False)

        w_min = max(self._gripper.width_min, 1e-4)
        w_max = self._gripper.width_max

        out: list[_Candidate] = []
        for i in sample_idx:
            pi = points[i]
            ni = normals[i]
            diff = points - pi                                 # (N, 3)
            dist = np.linalg.norm(diff, axis=1)                # (N,)
            width_mask = (dist >= w_min) & (dist <= w_max)
            if not width_mask.any():
                continue
            js = np.where(width_mask)[0]
            d = diff[js] / dist[js, None]
            align_i = np.abs(d @ ni)
            align_j = np.abs(np.einsum('kd,kd->k', normals[js], -d))
            score = np.minimum(align_i, align_j)
            keep = score >= self._antipodal_threshold
            for k in np.where(keep)[0]:
                j = int(js[k])
                if j <= int(i):
                    continue  # dedup ordered pairs
                out.append(_Candidate(
                    pi=pi, pj=points[j],
                    ni=ni, nj=normals[j],
                    width=float(dist[j]),
                    antipodal=float(score[k]),
                ))
        return out

    def _check_feasibility(
        self, candidates: list[_Candidate], scene: np.ndarray,
    ) -> list[_Candidate]:
        feasible: list[_Candidate] = []
        # Approach hints tried per candidate (in scene frame).
        hints = [
            np.array([0.0, 0.0, -1.0]),   # from above
            np.array([0.0, 0.0, +1.0]),   # from below
        ]
        fallback = np.array([1.0, 0.0, 0.0])

        for cand in candidates:
            closing = cand.closing_axis
            for hint in hints:
                R = frame_from_axes(closing, hint)
                if R is None:
                    R = frame_from_axes(closing, fallback)
                    if R is None:
                        continue
                T = np.eye(4)
                T[:3, :3] = R
                T[:3, 3] = cand.center

                grip_pts = self._gripper.collision_points(T, cand.width)
                clearance = _clearance_fraction(
                    grip_pts, scene, self._clearance_radius,
                )
                if clearance < self._min_clearance:
                    continue
                feasible.append(_Candidate(
                    pi=cand.pi, pj=cand.pj, ni=cand.ni, nj=cand.nj,
                    width=cand.width, antipodal=cand.antipodal,
                    transform=T, clearance=float(clearance),
                ))
        return feasible

    def _select(
        self, candidates: list[_Candidate], strategy: str,
    ) -> _Candidate:
        target_frac = _STRATEGY_WIDTH_FRACTION.get(strategy, 0.5)
        target_w = target_frac * self._gripper.width_max
        w_range = max(self._gripper.width_max - self._gripper.width_min, 1e-6)

        best: _Candidate | None = None
        for cand in candidates:
            cand.strategy_fit = float(
                max(0.0, 1.0 - abs(cand.width - target_w) / w_range)
            )
            cand.final_score = float(
                self._w_antipodal * cand.antipodal
                + self._w_clearance * cand.clearance
                + self._w_strategy * cand.strategy_fit
            )
            if best is None or cand.final_score > best.final_score:
                best = cand
        assert best is not None
        return best


# ---------------------------------------------------------------------------
# Module-level helpers
# ---------------------------------------------------------------------------

def _fail(message: str) -> PlanGrasp.Result:
    r = PlanGrasp.Result()
    r.success = False
    r.message = f'AntipodalPlanner: {message}'
    return r


def _crop_sphere(
    cloud: np.ndarray, center: np.ndarray, radius: float,
) -> np.ndarray:
    if cloud.shape[0] == 0:
        return cloud
    diff = cloud - center
    sq = np.einsum('ij,ij->i', diff, diff)
    return cloud[sq <= radius * radius]


def _random_downsample(
    cloud: np.ndarray, max_points: int, rng: np.random.Generator,
) -> np.ndarray:
    if cloud.shape[0] <= max_points:
        return cloud
    idx = rng.choice(cloud.shape[0], size=max_points, replace=False)
    return cloud[idx]


def _clearance_fraction(
    gripper_pts: np.ndarray, scene: np.ndarray, radius: float,
) -> float:
    """Fraction of gripper points whose nearest scene point is > ``radius``."""
    if gripper_pts.shape[0] == 0:
        return 1.0
    if scene.shape[0] == 0:
        return 1.0

    # Chunked nearest-neighbour query to bound memory.
    chunk = 256
    free = 0
    r2 = radius * radius
    for start in range(0, gripper_pts.shape[0], chunk):
        end = start + chunk
        g = gripper_pts[start:end]                         # (c, 3)
        diff = g[:, None, :] - scene[None, :, :]           # (c, M, 3)
        sq = np.einsum('cmd,cmd->cm', diff, diff)
        min_sq = sq.min(axis=1)                            # (c,)
        free += int((min_sq > r2).sum())
    return free / gripper_pts.shape[0]
