# A pose observation associated with a global object identity across cameras.
# Produced by cross_camera_associator, consumed by pose_filter_cpp.

int32 global_id                    # Cross-camera consistent ID
string class_name                  # YOLO class name
geometry_msgs/PoseStamped pose     # Pose in base_frame (ur5e_base_link)
float32 confidence                 # Weighted average confidence
int32 num_observing_cameras        # How many cameras observe this object
int32[] source_camera_ids          # Camera IDs that observe this object
int32[] source_local_ids           # Per-camera local object IDs
float32[] source_fitness_scores    # Per-camera ICP fitness scores
