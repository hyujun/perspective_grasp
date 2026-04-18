"""Phase 1 bringup: YOLO ByteTrack + PCL ICP pose estimator + pose filter."""

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    tracker_pkg = get_package_share_directory('yolo_pcl_cpp_tracker')
    filter_pkg = get_package_share_directory('pose_filter_cpp')

    tracker_config = os.path.join(tracker_pkg, 'config', 'tracker_params.yaml')
    filter_config = os.path.join(filter_pkg, 'config', 'filter_params.yaml')

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
            parameters=[tracker_config],
            output='screen',
        ),
        # C++ IEKF pose filter
        Node(
            package='pose_filter_cpp',
            executable='pose_filter_node',
            name='pose_filter',
            parameters=[filter_config],
            output='screen',
        ),
    ])
