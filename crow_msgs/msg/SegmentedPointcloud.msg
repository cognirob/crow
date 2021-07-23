# This msg wraps (segmented) pointcloud + its label
std_msgs/Header header
sensor_msgs/PointCloud2 pcl
uint16 object_id # Unique to each detection (also 1 person 1 id)
string label #must be one of the class labels from detector
float32 confidence #confidence, can be empty
