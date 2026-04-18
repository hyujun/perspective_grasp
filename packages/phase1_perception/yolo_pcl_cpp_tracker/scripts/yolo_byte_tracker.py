#!/usr/bin/env python3
"""YOLO + ByteTrack 2D multi-object detection and tracking node.

Subscribes to camera images, runs YOLOv8/v11 detection with ByteTrack
multi-object tracking, and publishes DetectionArray with persistent track IDs.
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Image
from perception_msgs.msg import Detection, DetectionArray
from cv_bridge import CvBridge

try:
    from ultralytics import YOLO
    HAS_ULTRALYTICS = True
except ImportError:
    HAS_ULTRALYTICS = False


class YoloByteTrackerNode(Node):
    """ROS 2 node wrapping YOLOv8/v11 + ByteTrack for 2D detection/tracking."""

    def __init__(self):
        super().__init__('yolo_byte_tracker')

        # Parameters
        self.declare_parameter('model_path', 'yolov8n.pt')
        self.declare_parameter('confidence_threshold', 0.5)
        self.declare_parameter('device', '0')  # GPU device ID
        self.declare_parameter('image_topic', '/camera/color/image_raw')
        self.declare_parameter('track_buffer', 30)
        self.declare_parameter('track_thresh', 0.5)
        self.declare_parameter('match_thresh', 0.8)

        model_path = self.get_parameter('model_path').value
        self.conf_thresh = self.get_parameter('confidence_threshold').value
        self.device = self.get_parameter('device').value
        image_topic = self.get_parameter('image_topic').value

        # CV Bridge
        self.bridge = CvBridge()

        # Load YOLO model
        if HAS_ULTRALYTICS:
            self.get_logger().info(f'Loading YOLO model: {model_path}')
            self.model = YOLO(model_path)
            self.get_logger().info('YOLO model loaded successfully')
        else:
            self.get_logger().error(
                'ultralytics not installed! pip3 install ultralytics')
            self.model = None

        # QoS: best effort, latest only
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        # Subscriber
        self.image_sub = self.create_subscription(
            Image, image_topic, self.image_callback, sensor_qos)

        # Publisher
        self.det_pub = self.create_publisher(
            DetectionArray, 'yolo/detections', sensor_qos)

        self.get_logger().info(
            f'YoloByteTracker ready. Subscribing to {image_topic}')

    def image_callback(self, msg: Image):
        """Process incoming image: run YOLO + ByteTrack, publish detections."""
        if self.model is None:
            return

        # Convert ROS Image to OpenCV
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        except Exception as e:
            self.get_logger().error(f'cv_bridge error: {e}')
            return

        # Run YOLO with ByteTrack tracking
        results = self.model.track(
            frame,
            persist=True,
            conf=self.conf_thresh,
            device=self.device,
            tracker='bytetrack.yaml',
            verbose=False,
        )

        # Build DetectionArray message
        det_array = DetectionArray()
        det_array.header = msg.header

        if results and len(results) > 0 and results[0].boxes is not None:
            boxes = results[0].boxes
            for box in boxes:
                det = Detection()

                # Track ID (ByteTrack assigns these)
                if box.id is not None:
                    det.id = int(box.id.item())
                else:
                    det.id = -1  # No track assigned yet

                # Class name
                cls_idx = int(box.cls.item())
                det.class_name = self.model.names.get(cls_idx, str(cls_idx))

                # Bounding box (xyxy format)
                x1, y1, x2, y2 = box.xyxy[0].cpu().numpy()
                det.bbox.x_offset = max(0, int(x1))
                det.bbox.y_offset = max(0, int(y1))
                det.bbox.width = max(1, int(x2 - x1))
                det.bbox.height = max(1, int(y2 - y1))
                det.bbox.do_rectify = False

                # Confidence
                det.confidence = float(box.conf.item())

                det_array.detections.append(det)

        self.det_pub.publish(det_array)


def main(args=None):
    rclpy.init(args=args)
    node = YoloByteTrackerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
