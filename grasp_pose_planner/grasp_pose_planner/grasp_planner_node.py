"""Grasp pose planning action server stub."""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer
from geometry_msgs.msg import PoseStamped
from perception_msgs.action import PlanGrasp


class GraspPlannerNode(Node):
    """Action server for grasp pose planning."""

    def __init__(self):
        super().__init__('grasp_planner')
        self.get_logger().info('GraspPlannerNode created (stub - not yet implemented)')

        self._action_server = ActionServer(
            self, PlanGrasp, 'plan_grasp',
            self._execute_callback)

    def _execute_callback(self, goal_handle):
        self.get_logger().info(
            f'PlanGrasp goal received: object_id={goal_handle.request.target_object_id}, '
            f'strategy={goal_handle.request.grasp_strategy} (stub)')

        # Publish feedback
        feedback = PlanGrasp.Feedback()
        feedback.num_candidates = 0
        feedback.best_score_so_far = 0.0
        feedback.current_stage = 'stub'
        goal_handle.publish_feedback(feedback)

        # Return empty result
        goal_handle.succeed()
        result = PlanGrasp.Result()
        result.grasp_pose = PoseStamped()
        result.finger_joint_config = []
        result.approach_waypoints = []
        result.grasp_quality_score = 0.0
        result.success = False
        result.message = 'Grasp planner not yet implemented'
        return result


def main(args=None):
    rclpy.init(args=args)
    node = GraspPlannerNode()
    node.get_logger().info('grasp_planner_node not yet implemented - spinning idle')
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
