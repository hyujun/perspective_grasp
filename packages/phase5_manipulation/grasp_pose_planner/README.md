# grasp_pose_planner

ROS 2 Action Server for grasp planning on a UR5e + 10-DoF hand. Pluggable **planner** × **gripper** adapters keep the algorithm robot-agnostic.

## Overview

Caches the latest `PoseWithMetaArray` (tracked objects) and `PointCloud2` (scene geometry) via subscription callbacks, then runs on-demand grasp planning per `plan_grasp` action goal. The current shipping planner is **antipodal** — samples candidate contact pairs, scores by antipodal quality + gripper clearance + strategy bias, returns the top grasp with approach waypoints.

| Component | Keys registered |
|---|---|
| Planner | `antipodal` |
| Gripper | `hand_10dof` |

Add a new planner by implementing `planners/base_planner.py::BasePlanner` and registering it in `PLANNER_REGISTRY`. Add a new gripper by implementing `grippers/base_gripper.py::BaseGripper` and registering it in `GRIPPER_REGISTRY`.

## Status

- ✓ Antipodal planner shipping (sample → score → top-k feasibility → best grasp).
- ✓ `Hand10DoF` gripper adapter with palm-box + finger-rail collision geometry.
- ⚠ `Hand10DoF.joint_config` preshapes (`power` / `precision` / `pinch`) are placeholder `[0.0] * 10`. Real 10-DoF URDF joint ordering + per-strategy preshapes + `width → joint` interpolation are **open TODOs** in [`grippers/hand_10dof.py`](grasp_pose_planner/grippers/hand_10dof.py). The planner runs end-to-end and publishes correct 6D grasp poses + approach waypoints; only the finger-joint vector is a stand-in.

## Node

**Node Name**: `grasp_planner`

| | |
|---|---|
| **Subscribes** | `pose_topic` (`perception_msgs/PoseWithMetaArray`), `cloud_topic` (`sensor_msgs/PointCloud2`) |
| **Action Server** | `plan_grasp` (`perception_msgs/action/PlanGrasp`) |

### `PlanGrasp` action

- **Goal**: `target_object_id` (int32), `grasp_strategy` (`power` / `precision` / `pinch`)
- **Feedback**: `num_candidates`, `best_score`, stage label (`generation` / `feasibility` / `selection`)
- **Result**: `grasp_pose` (`PoseStamped`), `finger_joints` (float32[] — 10-DoF preshape), `approach_waypoints` (`PoseStamped[]`), `quality_score`, `success`, `message`

## Parameters

| Parameter | Default | Description |
|---|---|---|
| `planner_type` | `antipodal` | Planner registry key |
| `gripper_type` | `hand_10dof` | Gripper registry key |
| `pose_topic` | `/pose_filter/filtered_poses` | Tracked-object pose stream |
| `cloud_topic` | `/merged/points` | Scene point cloud |
| `max_pose_age_sec` | `1.0` | Reject stale poses older than this at goal time |
| `max_cloud_age_sec` | `1.0` | Reject stale clouds older than this at goal time |
| `num_grasp_candidates` | `100` | Points sampled per planning call |
| `antipodal_threshold` | `0.85` | Min \|n·d\| for a valid antipodal pair |
| `approach_distance` | `0.10` | Pre-grasp offset along −approach axis (m) |
| `object_crop_radius` | `0.20` | Sphere radius around target pose (m) |
| `min_object_points` | `80` | Skip planning if crop has fewer points |
| `max_cloud_points` | `2000` | Downsample crop to this many points |
| `normals_knn` | `12` | k for PCA normal estimation |
| `top_k_feasibility` | `40` | Run clearance check on top-K pairs |
| `clearance_radius` | `0.008` | Gripper point → scene point threshold (m) |
| `min_clearance_fraction` | `0.85` | Fraction of gripper points that must be free of scene |
| `score_weight_antipodal` / `_clearance` / `_strategy` | `0.5` / `0.3` / `0.2` | Score mixing weights |
| `rng_seed` | `0` | `0` → nondeterministic |
| `gripper.width_min/max` | `0.0` / `0.12` m | Opening limits (Hand10DoF) |
| `gripper.palm_depth/height` | `0.05` / `0.06` m | Palm box dimensions (Hand10DoF) |
| `gripper.finger_length/thickness` | `0.09` / `0.02` m | Finger rail dimensions (Hand10DoF) |

Gripper dimensions above are **placeholder** — tune to the real hand before live manipulation. See [`config/grasp_planner_params.yaml`](config/grasp_planner_params.yaml).

## Running

```bash
colcon build --packages-select grasp_pose_planner
source install/setup.bash
ros2 launch grasp_pose_planner grasp_planner.launch.py

# Trigger a grasp plan (requires /pose_filter/filtered_poses + /merged/points to be publishing):
ros2 action send_goal /plan_grasp perception_msgs/action/PlanGrasp \
  "{target_object_id: 0, grasp_strategy: 'power'}" --feedback
```

## Frame conventions

All grasp poses and approach waypoints are published in the frame of the incoming `PoseWithMetaArray` (normally `ur5e_base_link` after Phase 3 filtering). Gripper geometry is sampled in the gripper's own frame:

- `approach_axis = +Z` — gripper approaches the target along +Z
- `closing_axis = ±Y` — fingers close along ±Y

## Dependencies

- **ROS**: `rclpy`, `sensor_msgs`, `geometry_msgs`, `perception_msgs`
- **Python**: numpy, scipy
