"""Launch YOLO ByteTrack detector + PCL ICP pose estimator."""

from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def _default_models_dir(anchor_pkg: str) -> str:
    """Resolve the repo's models/ directory.

    Overridable via PERSPECTIVE_GRASP_MODELS_DIR. Fallback walks up from the
    anchor package's share dir (install/<pkg>/share/<pkg>) to the colcon
    workspace root, then into src/perspective_grasp/models.
    """
    env = os.environ.get('PERSPECTIVE_GRASP_MODELS_DIR')
    if env:
        return env
    share = get_package_share_directory(anchor_pkg)
    ws_root = os.path.abspath(os.path.join(share, '..', '..', '..', '..'))
    return os.path.join(ws_root, 'src', 'perspective_grasp', 'models')


def generate_launch_description():
    pkg_dir = get_package_share_directory('yolo_pcl_cpp_tracker')
    config = os.path.join(pkg_dir, 'config', 'tracker_params.yaml')
    models_dir = _default_models_dir('yolo_pcl_cpp_tracker')

    return LaunchDescription([
        # Python YOLO + ByteTrack node
        Node(
            package='yolo_pcl_cpp_tracker',
            executable='yolo_byte_tracker.py',
            name='yolo_byte_tracker',
            output='screen',
            parameters=[{
                'model_path': 'yolov8n.pt',
                'models_dir': models_dir,
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
