cmake_minimum_required(VERSION 3.20)
project(perception_msgs VERSION 0.1.0 LANGUAGES C CXX)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(action_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  # Messages
  "msg/Detection.msg"
  "msg/DetectionArray.msg"
  "msg/PoseWithMeta.msg"
  "msg/PoseWithMetaArray.msg"
  "msg/PoseCovarianceStamped.msg"
  "msg/PoseCovarianceArray.msg"
  "msg/Segmentation.msg"
  "msg/SegmentationArray.msg"
  "msg/ObjectRelation.msg"
  "msg/PipelineStatus.msg"
  "msg/AssociatedPose.msg"
  "msg/AssociatedPoseArray.msg"
  # Services
  "srv/SetMode.srv"
  # Actions
  "action/AnalyzeScene.action"
  "action/PlanGrasp.action"
  DEPENDENCIES std_msgs geometry_msgs sensor_msgs action_msgs
)

ament_package()
