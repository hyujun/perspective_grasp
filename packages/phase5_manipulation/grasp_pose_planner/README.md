# grasp_pose_planner

ROS 2 action server for grasp pose planning targeting a 10-DoF robotic hand on a UR5e.

## Overview

Computes optimal grasp poses for robotic manipulation. Accepts a target object ID and grasp strategy, then generates, evaluates, and selects the best grasp candidate with approach trajectory waypoints.

**Status**: Stub implementation — action server skeleton is in place, grasp planning algorithms are pending.

## Node

**Node Name**: `grasp_planner`

| | |
|---|---|
| **Action Server** | `plan_grasp` (`PlanGrasp`) |

### PlanGrasp Action

- **Goal**: `target_object_id` (int32), `grasp_strategy` (power / precision / pinch)
- **Result**: Grasp pose, 10-DoF finger joint config, approach waypoints, quality score, success flag
- **Feedback**: Number of candidates, best score, current stage (generation / feasibility / selection)

## Parameters

| Parameter | Default | Description |
|-----------|---------|-------------|
| `num_grasp_candidates` | `100` | Number of candidates to generate |
| `antipodal_threshold` | `0.7` | Antipodal grasp quality threshold |
| `collision_check` | `true` | Enable collision checking |
| `approach_distance` | `0.10` | Approach trajectory distance (m) |
| `gripper_width_max` | `0.085` | Maximum gripper opening width (m) |

## Dependencies

- `rclpy`, `geometry_msgs`, `perception_msgs`

## Build

```bash
colcon build --packages-select grasp_pose_planner
```
