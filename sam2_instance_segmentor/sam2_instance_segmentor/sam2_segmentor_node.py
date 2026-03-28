"""SAM2 instance segmentation lifecycle node stub."""

import rclpy
from rclpy.lifecycle import LifecycleNode, LifecycleState, TransitionCallbackReturn
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image
from perception_msgs.msg import DetectionArray, SegmentationArray


class Sam2SegmentorNode(LifecycleNode):
    """LifecycleNode for SAM2 instance segmentation."""

    def __init__(self):
        super().__init__('sam2_segmentor')
        self.get_logger().info('Sam2SegmentorNode created (stub - not yet implemented)')

    def on_configure(self, state: LifecycleState) -> TransitionCallbackReturn:
        self.get_logger().info('Configuring Sam2SegmentorNode')

        sensor_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)

        self._sub_image = self.create_subscription(
            Image, '/camera/color/image_raw',
            self._image_callback, sensor_qos)
        self._sub_detections = self.create_subscription(
            DetectionArray, '/yolo/detections',
            self._detections_callback, sensor_qos)
        self._pub_masks = self.create_publisher(
            SegmentationArray, '/sam2/masks', sensor_qos)

        return TransitionCallbackReturn.SUCCESS

    def on_activate(self, state: LifecycleState) -> TransitionCallbackReturn:
        self.get_logger().info('Activating Sam2SegmentorNode')
        return super().on_activate(state)

    def on_deactivate(self, state: LifecycleState) -> TransitionCallbackReturn:
        self.get_logger().info('Deactivating Sam2SegmentorNode')
        return super().on_deactivate(state)

    def on_cleanup(self, state: LifecycleState) -> TransitionCallbackReturn:
        self.get_logger().info('Cleaning up Sam2SegmentorNode')
        return TransitionCallbackReturn.SUCCESS

    def _image_callback(self, msg: Image):
        pass  # TODO: run SAM2 inference

    def _detections_callback(self, msg: DetectionArray):
        pass  # TODO: use YOLO boxes as SAM2 prompts


def main(args=None):
    rclpy.init(args=args)
    node = Sam2SegmentorNode()
    node.get_logger().info('sam2_segmentor_node not yet implemented - spinning idle')
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
