"""Launch YOLO ByteTrack detector + PCL ICP pose estimator."""

from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    pkg_dir = get_package_share_directory('yolo_pcl_cpp_tracker')
    config = os.path.join(pkg_dir, 'config', 'tracker_params.yaml')

    return LaunchDescription([
        # Python YOLO + ByteTrack node
        Node(
            package='yolo_pcl_cpp_tracker',
            executable='yolo_byte_tracker.py',
            name='yolo_byte_tracker',
            output='screen',
            parameters=[{
                'model_path': 'yolov8n.pt',
                'confidence_threshold': 0.5,
                'image_topic': '/camera/color/image_raw',
            }],
        ),
        # C++ PCL ICP pose estimator
        Node(
            package='yolo_pcl_cpp_tracker',
            executable='pcl_icp_pose_estimator',
            name='pcl_icp_pose_estimator',
            parameters=[config],
            output='screen',
        ),
    ])
