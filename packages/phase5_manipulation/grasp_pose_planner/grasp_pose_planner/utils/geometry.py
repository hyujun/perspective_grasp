"""Numpy helpers: pose conversions, normals, axis construction."""

from __future__ import annotations

import math

import numpy as np
from geometry_msgs.msg import Pose, PoseStamped, Quaternion


def pose_to_matrix(pose: Pose) -> np.ndarray:
    """geometry_msgs/Pose → 4x4 homogeneous transform."""
    q = pose.orientation
    t = pose.position
    R = quaternion_to_matrix(q.x, q.y, q.z, q.w)
    T = np.eye(4)
    T[:3, :3] = R
    T[:3, 3] = [t.x, t.y, t.z]
    return T


def matrix_to_pose(T: np.ndarray) -> Pose:
    """4x4 homogeneous transform → geometry_msgs/Pose."""
    pose = Pose()
    pose.position.x, pose.position.y, pose.position.z = (float(v) for v in T[:3, 3])
    qx, qy, qz, qw = matrix_to_quaternion(T[:3, :3])
    pose.orientation = Quaternion(x=qx, y=qy, z=qz, w=qw)
    return pose


def quaternion_to_matrix(x: float, y: float, z: float, w: float) -> np.ndarray:
    n = math.sqrt(x * x + y * y + z * z + w * w)
    if n < 1e-12:
        return np.eye(3)
    x, y, z, w = x / n, y / n, z / n, w / n
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w),     2 * (x * z + y * w)],
        [2 * (x * y + z * w),     1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w),     2 * (y * z + x * w),     1 - 2 * (x * x + y * y)],
    ])


def matrix_to_quaternion(R: np.ndarray) -> tuple[float, float, float, float]:
    """3x3 rotation → (x, y, z, w). Shepperd's method."""
    trace = R[0, 0] + R[1, 1] + R[2, 2]
    if trace > 0.0:
        s = 0.5 / math.sqrt(trace + 1.0)
        w = 0.25 / s
        x = (R[2, 1] - R[1, 2]) * s
        y = (R[0, 2] - R[2, 0]) * s
        z = (R[1, 0] - R[0, 1]) * s
    elif R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
        s = 2.0 * math.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2])
        w = (R[2, 1] - R[1, 2]) / s
        x = 0.25 * s
        y = (R[0, 1] + R[1, 0]) / s
        z = (R[0, 2] + R[2, 0]) / s
    elif R[1, 1] > R[2, 2]:
        s = 2.0 * math.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2])
        w = (R[0, 2] - R[2, 0]) / s
        x = (R[0, 1] + R[1, 0]) / s
        y = 0.25 * s
        z = (R[1, 2] + R[2, 1]) / s
    else:
        s = 2.0 * math.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1])
        w = (R[1, 0] - R[0, 1]) / s
        x = (R[0, 2] + R[2, 0]) / s
        y = (R[1, 2] + R[2, 1]) / s
        z = 0.25 * s
    return float(x), float(y), float(z), float(w)


def frame_from_axes(
    closing: np.ndarray, approach_hint: np.ndarray,
) -> np.ndarray | None:
    """Build an orthonormal rotation from a closing axis and approach hint.

    Gripper frame convention matches ``BaseGripper``:
      * ``closing`` → gripper's Y axis (fingers close along this)
      * approach   → gripper's Z axis (palm-to-tip)
      * X axis     = Y × Z  (right-handed)

    ``approach_hint`` is projected onto the plane perpendicular to the
    closing axis. Returns ``None`` if the hint is colinear with the
    closing axis (degenerate — caller should try a different hint).
    """
    y = closing / (np.linalg.norm(closing) + 1e-12)
    z_raw = approach_hint - np.dot(approach_hint, y) * y
    norm = np.linalg.norm(z_raw)
    if norm < 1e-6:
        return None
    z = z_raw / norm
    x = np.cross(y, z)
    x /= np.linalg.norm(x) + 1e-12
    return np.stack([x, y, z], axis=1)  # columns are basis vectors


def compute_normals(points: np.ndarray, k: int = 10) -> np.ndarray:
    """Per-point normals via k-NN PCA.

    Orientation is not resolved (normal and -normal are both valid); the
    planner only uses ``|n · d|`` so this does not matter.

    Parameters
    ----------
    points : (N, 3) float array
    k      : neighbourhood size

    Returns
    -------
    normals : (N, 3) float array
    """
    n = points.shape[0]
    if n == 0:
        return np.zeros((0, 3))
    k = max(3, min(k, n - 1))

    # Pairwise squared distance — OK for N up to a few thousand.
    diff = points[:, None, :] - points[None, :, :]
    sq = np.einsum('ijk,ijk->ij', diff, diff)
    # k+1 because the point itself is at distance 0.
    idx = np.argpartition(sq, kth=k, axis=1)[:, : k + 1]

    normals = np.zeros_like(points)
    for i in range(n):
        neigh = points[idx[i]]
        centred = neigh - neigh.mean(axis=0)
        cov = centred.T @ centred
        eigvals, eigvecs = np.linalg.eigh(cov)
        normals[i] = eigvecs[:, 0]  # smallest eigenvalue → surface normal
    return normals


def build_pose_stamped(
    T: np.ndarray, frame_id: str, stamp,
) -> PoseStamped:
    """4x4 transform → PoseStamped in the given frame."""
    ps = PoseStamped()
    ps.header.frame_id = frame_id
    ps.header.stamp = stamp
    ps.pose = matrix_to_pose(T)
    return ps
