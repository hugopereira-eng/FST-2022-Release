std_msgs/Header header          # cone header
geometry_msgs/Point position    # coordinate of cone in [x, y, z]
int32 color                     # color of cone, 0 = blue, 1 = yellow, 2 = orange, 3 = big-orange, 4 = unknown
float32 probability             # cone color probability
int32 id                        # cone ID from tracking
pcl_msgs/PointIndices point_indices # pointcloud points of the cone