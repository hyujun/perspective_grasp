# 6D pose with metadata from a specific estimator source
int32 object_id
string class_name
string source                            # yolo_tracker / foundationpose / megapose / bundlesdf
geometry_msgs/PoseStamped pose
float32 confidence
float32 fitness_score
