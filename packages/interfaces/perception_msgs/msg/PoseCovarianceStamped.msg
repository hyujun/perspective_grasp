# Filtered pose with 6x6 covariance on SE(3) tangent space
int32 object_id
geometry_msgs/PoseStamped pose
float64[36] covariance                   # 6x6 row-major [rot_x, rot_y, rot_z, trans_x, trans_y, trans_z]
