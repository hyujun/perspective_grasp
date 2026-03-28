"""Isaac ROS FoundationPose lifecycle node stub."""

import rclpy
from rclpy.lifecycle import LifecycleNode, LifecycleState, TransitionCallbackReturn
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image
from perception_msgs.msg import DetectionArray, PoseWithMetaArray


class FoundationPoseNode(LifecycleNode):
    """LifecycleNode wrapper for Isaac ROS FoundationPose."""

    def __init__(self):
        super().__init__('foundationpose_tracker')
        self.get_logger().info('FoundationPoseNode created (stub - not yet implemented)')

    def on_configure(self, state: LifecycleState) -> TransitionCallbackReturn:
        self.get_logger().info('Configuring FoundationPoseNode')

        sensor_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)

        self._sub_image = self.create_subscription(
            Image, '/camera/color/image_raw',
            self._image_callback, sensor_qos)
        self._sub_detections = self.create_subscription(
            DetectionArray, '/yolo/detections',
            self._detections_callback, sensor_qos)
        self._pub_poses = self.create_publisher(
            PoseWithMetaArray, '/foundationpose/raw_poses', sensor_qos)

        return TransitionCallbackReturn.SUCCESS

    def on_activate(self, state: LifecycleState) -> TransitionCallbackReturn:
        self.get_logger().info('Activating FoundationPoseNode')
        return super().on_activate(state)

    def on_deactivate(self, state: LifecycleState) -> TransitionCallbackReturn:
        self.get_logger().info('Deactivating FoundationPoseNode')
        return super().on_deactivate(state)

    def on_cleanup(self, state: LifecycleState) -> TransitionCallbackReturn:
        self.get_logger().info('Cleaning up FoundationPoseNode')
        return TransitionCallbackReturn.SUCCESS

    def _image_callback(self, msg: Image):
        pass  # TODO: forward to FoundationPose

    def _detections_callback(self, msg: DetectionArray):
        pass  # TODO: use detections for region-of-interest


def main(args=None):
    rclpy.init(args=args)
    node = FoundationPoseNode()
    node.get_logger().info('foundationpose_node not yet implemented - spinning idle')
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
