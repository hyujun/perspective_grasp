"""CosyPose scene optimization action server stub."""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer
from rclpy.qos import QoSProfile, ReliabilityPolicy
from perception_msgs.action import AnalyzeScene
from perception_msgs.msg import PoseWithMetaArray


class CosyPoseNode(Node):
    """Action server for CosyPose scene-level pose optimization."""

    def __init__(self):
        super().__init__('cosypose_optimizer')
        self.get_logger().info('CosyPoseNode created (stub - not yet implemented)')

        sensor_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)

        self._action_server = ActionServer(
            self, AnalyzeScene, 'analyze_scene',
            self._execute_callback)

        self._pub_optimized = self.create_publisher(
            PoseWithMetaArray, '/cosypose/optimized_poses', sensor_qos)

    def _execute_callback(self, goal_handle):
        self.get_logger().info('AnalyzeScene goal received (stub - returning empty result)')

        # Publish feedback
        feedback = AnalyzeScene.Feedback()
        feedback.progress = 0.0
        feedback.current_stage = 'stub'
        goal_handle.publish_feedback(feedback)

        # Return empty result
        goal_handle.succeed()
        result = AnalyzeScene.Result()
        result.optimized_poses = []
        result.relations = []
        result.success = False
        result.message = 'CosyPose not yet implemented'
        return result


def main(args=None):
    rclpy.init(args=args)
    node = CosyPoseNode()
    node.get_logger().info('cosypose_node not yet implemented - spinning idle')
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
