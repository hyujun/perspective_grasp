#!/usr/bin/env python3
# Copyright 2026 perspective_grasp
# SPDX-License-Identifier: Apache-2.0

"""
ROS 2 node for online calibration data collection.

Subscribes to camera images (per camera) and robot joint states.
Provides a /calibration/capture service to trigger sample capture.
Detects ChArUco board in each camera image and records robot FK.
Saves samples to YAML files in an output directory.
"""

import os
import time
from pathlib import Path

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import Image, JointState
from std_srvs.srv import Trigger


class CalibrationDataCollector(Node):
    def __init__(self):
        super().__init__("calibration_data_collector")

        # Declare parameters
        self.declare_parameter("num_cameras", 3)
        self.declare_parameter("camera_namespaces", ["/cam0", "/cam1", "/cam2"])
        self.declare_parameter("joint_state_topic", "/joint_states")
        self.declare_parameter("output_dir", "~/calibration_results")
        self.declare_parameter("charuco.squares_x", 7)
        self.declare_parameter("charuco.squares_y", 5)
        self.declare_parameter("charuco.square_length", 0.04)
        self.declare_parameter("charuco.marker_length", 0.03)
        self.declare_parameter("charuco.dictionary", "DICT_6X6_250")
        self.declare_parameter("charuco.min_corners", 4)

        # Get parameters
        self.num_cameras = self.get_parameter("num_cameras").value
        self.camera_namespaces = self.get_parameter("camera_namespaces").value
        joint_state_topic = self.get_parameter("joint_state_topic").value
        output_dir = self.get_parameter("output_dir").value
        self.output_dir = Path(os.path.expanduser(output_dir))
        self.output_dir.mkdir(parents=True, exist_ok=True)

        # ChArUco board config
        squares_x = self.get_parameter("charuco.squares_x").value
        squares_y = self.get_parameter("charuco.squares_y").value
        square_length = self.get_parameter("charuco.square_length").value
        marker_length = self.get_parameter("charuco.marker_length").value
        dict_name = self.get_parameter("charuco.dictionary").value
        self.min_corners = self.get_parameter("charuco.min_corners").value

        # Create ChArUco board
        dict_id = getattr(cv2.aruco, dict_name, cv2.aruco.DICT_6X6_250)
        self.aruco_dict = cv2.aruco.getPredefinedDictionary(dict_id)
        self.charuco_board = cv2.aruco.CharucoBoard_create(
            squares_x, squares_y, square_length, marker_length, self.aruco_dict
        )
        self.detector_params = cv2.aruco.DetectorParameters_create()

        # State
        self.bridge = CvBridge()
        self.latest_images: dict[int, np.ndarray] = {}
        self.latest_joint_state: JointState | None = None
        self.samples: list[dict] = []

        # Subscribe to camera images
        self.image_subs = []
        for i, ns in enumerate(self.camera_namespaces[: self.num_cameras]):
            topic = f"{ns}/color/image_raw"
            sub = self.create_subscription(
                Image,
                topic,
                lambda msg, cam_id=i: self._image_callback(msg, cam_id),
                1,
            )
            self.image_subs.append(sub)
            self.get_logger().info(f"Subscribing to {topic}")

        # Subscribe to joint states
        self.joint_sub = self.create_subscription(
            JointState, joint_state_topic, self._joint_state_callback, 1
        )

        # Capture service
        self.capture_srv = self.create_service(
            Trigger, "/calibration/capture", self._capture_callback
        )
        self.get_logger().info(
            f"Calibration data collector ready. "
            f"Call /calibration/capture to capture samples. "
            f"Output: {self.output_dir}"
        )

    def _image_callback(self, msg: Image, cam_id: int):
        self.latest_images[cam_id] = self.bridge.imgmsg_to_cv2(msg, "bgr8")

    def _joint_state_callback(self, msg: JointState):
        self.latest_joint_state = msg

    def _capture_callback(self, request, response):
        if self.latest_joint_state is None:
            response.success = False
            response.message = "No joint state received yet"
            return response

        sample_id = len(self.samples)
        detected_cameras = 0
        sample_data = {
            "id": sample_id,
            "timestamp": time.time(),
            "joint_positions": list(self.latest_joint_state.position),
            "joint_names": list(self.latest_joint_state.name),
            "detections": {},
        }

        for cam_id in range(self.num_cameras):
            if cam_id not in self.latest_images:
                self.get_logger().warn(f"No image from camera {cam_id}")
                continue

            image = self.latest_images[cam_id]
            gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

            # Detect ArUco markers
            corners, ids, _ = cv2.aruco.detectMarkers(
                gray, self.aruco_dict, parameters=self.detector_params
            )

            if ids is None or len(ids) == 0:
                self.get_logger().warn(f"Camera {cam_id}: no markers detected")
                continue

            # Interpolate ChArUco corners
            num_corners, charuco_corners, charuco_ids = (
                cv2.aruco.interpolateCornersCharuco(
                    corners, ids, gray, self.charuco_board
                )
            )

            if num_corners < self.min_corners:
                self.get_logger().warn(
                    f"Camera {cam_id}: only {num_corners} corners "
                    f"(need {self.min_corners})"
                )
                continue

            detected_cameras += 1
            sample_data["detections"][cam_id] = {
                "num_corners": int(num_corners),
                "charuco_corners": charuco_corners.tolist(),
                "charuco_ids": charuco_ids.flatten().tolist(),
            }

            # Save annotated image
            img_path = self.output_dir / f"sample_{sample_id:04d}_cam{cam_id}.png"
            annotated = image.copy()
            cv2.aruco.drawDetectedCornersCharuco(
                annotated, charuco_corners, charuco_ids
            )
            cv2.imwrite(str(img_path), annotated)

        if detected_cameras == 0:
            response.success = False
            response.message = "No ChArUco board detected in any camera"
            return response

        self.samples.append(sample_data)

        # Save all samples to YAML
        self._save_samples()

        response.success = True
        response.message = (
            f"Sample {sample_id} captured: "
            f"{detected_cameras}/{self.num_cameras} cameras detected board. "
            f"Total samples: {len(self.samples)}"
        )
        self.get_logger().info(response.message)
        return response

    def _save_samples(self):
        """Save all collected samples to a YAML file using OpenCV FileStorage."""
        output_file = str(self.output_dir / "collected_samples.yaml")
        fs = cv2.FileStorage(output_file, cv2.FileStorage_WRITE)
        fs.write("num_samples", len(self.samples))
        for i, sample in enumerate(self.samples):
            fs.write(f"sample_{i}_id", sample["id"])
            fs.write(f"sample_{i}_timestamp", sample["timestamp"])
            fs.write(
                f"sample_{i}_joint_positions",
                np.array(sample["joint_positions"]),
            )
            for cam_id, det in sample["detections"].items():
                prefix = f"sample_{i}_cam_{cam_id}"
                fs.write(f"{prefix}_num_corners", det["num_corners"])
                fs.write(
                    f"{prefix}_charuco_corners",
                    np.array(det["charuco_corners"], dtype=np.float32),
                )
                fs.write(
                    f"{prefix}_charuco_ids",
                    np.array(det["charuco_ids"], dtype=np.int32),
                )
        fs.release()
        self.get_logger().info(f"Saved {len(self.samples)} samples to {output_file}")


def main(args=None):
    rclpy.init(args=args)
    node = CalibrationDataCollector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
