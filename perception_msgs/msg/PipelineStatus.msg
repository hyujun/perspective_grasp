# Current status of the perception meta controller
std_msgs/Header header
string active_mode                       # NORMAL / HIGH_PRECISION / SCENE_ANALYSIS
string[] active_nodes                    # list of currently active node names
float32 gpu_memory_usage_percent
int32 tracked_object_count
int32 lost_object_count
