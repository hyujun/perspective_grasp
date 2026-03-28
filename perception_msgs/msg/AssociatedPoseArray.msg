# Array of cross-camera associated poses.
# Published by cross_camera_associator at a fixed rate (e.g., 30 Hz).

std_msgs/Header header
perception_msgs/AssociatedPose[] poses
