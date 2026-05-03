"""Phase 1 bringup: YOLO ByteTrack + PCL ICP pose estimator + pose filter."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from perception_launch_utils import config_path, workspace_models_dir


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'image_qos',
            default_value='reliable',
            description=(
                "QoS for the image subscription. Default 'reliable' matches "
                'ros-jazzy realsense2_camera 4.57.7 (publishes RELIABLE). '
                "Switch to 'sensor_data' (BEST_EFFORT) for SensorDataQoS "
                'cameras.'
            ),
        ),
        # Python YOLO + ByteTrack node
        Node(
            package='yolo_pcl_cpp_tracker',
            executable='yolo_byte_tracker.py',
            name='yolo_byte_tracker',
            output='screen',
            parameters=[{
                'model_path': 'yolov8n.pt',
                'models_dir': workspace_models_dir('perception_bringup'),
                'confidence_threshold': 0.5,
                'image_topic': '/camera/color/image_raw',
                'image_qos': LaunchConfiguration('image_qos'),
            }],
        ),
        # C++ PCL ICP pose estimator
        Node(
            package='yolo_pcl_cpp_tracker',
            executable='pcl_icp_pose_estimator',
            name='pcl_icp_pose_estimator',
            parameters=[config_path(
                'yolo_pcl_cpp_tracker', 'tracker_params.yaml')],
            output='screen',
        ),
        # C++ IEKF pose filter
        Node(
            package='pose_filter_cpp',
            executable='pose_filter_node',
            name='pose_filter',
            parameters=[config_path(
                'pose_filter_cpp', 'filter_params.yaml')],
            output='screen',
        ),
    ])
