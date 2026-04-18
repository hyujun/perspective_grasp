"""Container-resident live test for Sam2SegmentorNode.

Assumes the node has already been transitioned to Active (drive lifecycle
externally). Publishes a synthetic Image + DetectionArray with multiple
boxes to exercise batched segmentation, then verifies:

  * each kept detection maps to a distinct mask with the same id
  * confidence-threshold filtering drops sub-threshold detections
    (pass ``--expected-threshold`` matching the node's
    ``min_detection_confidence`` parameter)
  * tight-bbox of each returned mask lies inside the requested prompt box

Use ``--single`` to fall back to the old 1-box smoke test.
"""

import argparse
import sys
import time
from dataclasses import dataclass

import numpy as np
import rclpy
from builtin_interfaces.msg import Time as TimeMsg
from cv_bridge import CvBridge
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Image, RegionOfInterest

from perception_msgs.msg import Detection, DetectionArray, SegmentationArray


@dataclass
class SyntheticObject:
    det_id: int
    class_name: str
    confidence: float
    bbox_xywh: tuple[int, int, int, int]  # x, y, w, h
    color: tuple[int, int, int]


def _stamp_now(node) -> TimeMsg:
    now_ns = node.get_clock().now().nanoseconds
    return TimeMsg(
        sec=int(now_ns // 1_000_000_000),
        nanosec=int(now_ns % 1_000_000_000),
    )


def _build_scene(objects: list[SyntheticObject],
                 h: int = 480, w: int = 640) -> np.ndarray:
    img = np.full((h, w, 3), 40, dtype=np.uint8)
    for obj in objects:
        x, y, bw, bh = obj.bbox_xywh
        img[y:y + bh, x:x + bw] = obj.color
    return img


def _build_detections(objects: list[SyntheticObject],
                      stamp: TimeMsg, frame_id: str) -> DetectionArray:
    msg = DetectionArray()
    msg.header.stamp = stamp
    msg.header.frame_id = frame_id
    for obj in objects:
        det = Detection(
            id=obj.det_id, class_name=obj.class_name,
            confidence=obj.confidence,
        )
        x, y, bw, bh = obj.bbox_xywh
        det.bbox = RegionOfInterest(
            x_offset=x, y_offset=y, width=bw, height=bh,
        )
        msg.detections.append(det)
    return msg


def _scenarios(multi: bool) -> list[SyntheticObject]:
    if not multi:
        return [SyntheticObject(
            det_id=7, class_name='test_obj', confidence=0.95,
            bbox_xywh=(215, 145, 220, 190), color=(220, 90, 60),
        )]
    # 4 detections:
    #  * two non-overlapping objects (left + right) at high conf
    #  * one overlapping the left object (partial occlusion scenario)
    #  * one at low conf — filtered iff the node is configured with
    #    min_detection_confidence above 0.10 (see --expected-threshold)
    return [
        SyntheticObject(
            det_id=1, class_name='box_left', confidence=0.92,
            bbox_xywh=(60, 150, 140, 180), color=(220, 90, 60),
        ),
        SyntheticObject(
            det_id=2, class_name='box_right', confidence=0.88,
            bbox_xywh=(420, 140, 150, 200), color=(60, 180, 220),
        ),
        SyntheticObject(
            det_id=3, class_name='box_left_inner', confidence=0.75,
            bbox_xywh=(90, 180, 80, 90), color=(40, 210, 80),
        ),
        SyntheticObject(
            det_id=99, class_name='low_conf', confidence=0.10,
            bbox_xywh=(260, 300, 100, 90), color=(180, 180, 40),
        ),
    ]


def _validate(arr: SegmentationArray,
              objects: list[SyntheticObject],
              threshold: float,
              bridge: CvBridge,
              logger) -> bool:
    expected_ids = {o.det_id for o in objects if o.confidence >= threshold}
    filtered_ids = {o.det_id for o in objects if o.confidence < threshold}
    got_ids = {seg.id for seg in arr.segmentations}

    ok = True
    missing = expected_ids - got_ids
    if missing:
        logger.error(f'missing kept ids: {sorted(missing)}')
        ok = False
    leaked = got_ids & filtered_ids
    if leaked:
        logger.error(f'ids that should have been filtered: {sorted(leaked)}')
        ok = False

    by_id = {o.det_id: o for o in objects}
    for seg in arr.segmentations:
        mask = bridge.imgmsg_to_cv2(seg.mask, desired_encoding='mono8')
        nz = int((mask > 0).sum())
        if nz == 0:
            logger.error(f'  id={seg.id}: empty mask')
            ok = False
            continue
        ys, xs = np.where(mask > 0)
        tight = (int(xs.min()), int(ys.min()),
                 int(xs.max()), int(ys.max()))
        logger.info(
            f'  id={seg.id} conf={seg.confidence:.3f} '
            f'mask_shape={mask.shape} nonzero={nz} tight_bbox={tight}'
        )
        obj = by_id.get(seg.id)
        if obj is None:
            logger.error(f'  id={seg.id} not in prompt set')
            ok = False
            continue
        x, y, bw, bh = obj.bbox_xywh
        px2, py2 = x + bw, y + bh
        # Allow small slop: masks can extend a couple px beyond the prompt box.
        slop = 6
        inside = (tight[0] >= x - slop and tight[1] >= y - slop
                  and tight[2] <= px2 + slop and tight[3] <= py2 + slop)
        if not inside:
            logger.error(
                f'  id={seg.id} tight_bbox {tight} escapes prompt '
                f'({x},{y},{px2},{py2})'
            )
            ok = False
    return ok


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--single', action='store_true',
                        help='use the old 1-detection scenario')
    parser.add_argument('--image-topic',
                        default='/camera/color/image_raw',
                        help='image topic to publish on')
    parser.add_argument('--detections-topic',
                        default='/yolo/detections',
                        help='detections topic to publish on')
    parser.add_argument('--masks-topic',
                        default='/sam2/masks',
                        help='masks topic to subscribe to')
    parser.add_argument('--expected-threshold', type=float, default=0.0,
                        help=('node-side min_detection_confidence the test '
                              'should assume when judging which detections '
                              'must come back. Match what the node was '
                              'started with.'))
    args = parser.parse_args()

    rclpy.init()
    node = rclpy.create_node('sam2_live_tester')
    bridge = CvBridge()
    qos = QoSProfile(
        reliability=ReliabilityPolicy.BEST_EFFORT,
        history=HistoryPolicy.KEEP_LAST,
        depth=1,
    )

    received: list[SegmentationArray] = []
    node.create_subscription(
        SegmentationArray, args.masks_topic,
        lambda m: received.append(m), qos,
    )
    img_pub = node.create_publisher(Image, args.image_topic, qos)
    det_pub = node.create_publisher(DetectionArray, args.detections_topic, qos)

    objects = _scenarios(multi=not args.single)
    img = _build_scene(objects)
    stamp = _stamp_now(node)
    image_msg = bridge.cv2_to_imgmsg(img, encoding='rgb8')
    image_msg.header.stamp = stamp
    image_msg.header.frame_id = 'test_cam'
    det_msg = _build_detections(objects, stamp, 'test_cam')

    deadline = time.time() + 10.0
    while time.time() < deadline:
        rclpy.spin_once(node, timeout_sec=0.05)
        if (img_pub.get_subscription_count() > 0
                and det_pub.get_subscription_count() > 0):
            break
    node.get_logger().info(
        f'pub discovery: image_subs={img_pub.get_subscription_count()}, '
        f'det_subs={det_pub.get_subscription_count()}'
    )

    kept_expected = sum(o.confidence >= args.expected_threshold
                        for o in objects)
    node.get_logger().info(
        f'publishing {len(objects)} detection(s); expect {kept_expected} '
        f'kept at threshold={args.expected_threshold:.2f}'
    )
    for _ in range(3):
        img_pub.publish(image_msg)
        time.sleep(0.05)
        det_pub.publish(det_msg)
        time.sleep(0.2)
        for _ in range(5):
            rclpy.spin_once(node, timeout_sec=0.02)
        if received:
            break

    deadline = time.time() + 30.0
    while time.time() < deadline and not received:
        rclpy.spin_once(node, timeout_sec=0.1)

    rc = 0
    if not received:
        node.get_logger().error('no SegmentationArray received within 30s')
        rc = 2
    else:
        arr = received[0]
        node.get_logger().info(f'got {len(arr.segmentations)} segmentation(s)')
        if not _validate(arr, objects, args.expected_threshold,
                         bridge, node.get_logger()):
            rc = 1

    node.destroy_node()
    rclpy.try_shutdown()
    return rc


if __name__ == '__main__':
    sys.exit(main())
