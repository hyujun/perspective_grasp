"""BundleSDF unknown object tracking lifecycle node stub."""

import rclpy
from rclpy.lifecycle import LifecycleNode, LifecycleState, TransitionCallbackReturn
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image, CameraInfo
from perception_msgs.msg import PoseWithMetaArray, SegmentationArray


class BundleSdfNode(LifecycleNode):
    """LifecycleNode for BundleSDF unknown object tracking and reconstruction."""

    def __init__(self):
        super().__init__('bundlesdf_tracker')
        self.get_logger().info('BundleSdfNode created (stub - not yet implemented)')

    def on_configure(self, state: LifecycleState) -> TransitionCallbackReturn:
        self.get_logger().info('Configuring BundleSdfNode')

        sensor_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)

        self._sub_color = self.create_subscription(
            Image, '/camera/color/image_raw',
            self._color_callback, sensor_qos)
        self._sub_depth = self.create_subscription(
            Image, '/camera/depth/image_rect_raw',
            self._depth_callback, sensor_qos)
        self._sub_masks = self.create_subscription(
            SegmentationArray, '/sam2/masks',
            self._masks_callback, sensor_qos)

        self._pub_poses = self.create_publisher(
            PoseWithMetaArray, '/bundlesdf/raw_poses', sensor_qos)

        return TransitionCallbackReturn.SUCCESS

    def on_activate(self, state: LifecycleState) -> TransitionCallbackReturn:
        self.get_logger().info('Activating BundleSdfNode')
        return super().on_activate(state)

    def on_deactivate(self, state: LifecycleState) -> TransitionCallbackReturn:
        self.get_logger().info('Deactivating BundleSdfNode')
        return super().on_deactivate(state)

    def on_cleanup(self, state: LifecycleState) -> TransitionCallbackReturn:
        self.get_logger().info('Cleaning up BundleSdfNode')
        return TransitionCallbackReturn.SUCCESS

    def _color_callback(self, msg: Image):
        pass  # TODO: buffer for BundleSDF

    def _depth_callback(self, msg: Image):
        pass  # TODO: buffer for BundleSDF

    def _masks_callback(self, msg: SegmentationArray):
        pass  # TODO: use masks for object segmentation


def main(args=None):
    rclpy.init(args=args)
    node = BundleSdfNode()
    node.get_logger().info('bundlesdf_node not yet implemented - spinning idle')
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
