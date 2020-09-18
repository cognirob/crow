# This msg wraps (segmented) pointcloud + its label
std_msgs/Header header
PointCloud2 pcl 
string label #must be one of the class labels from detector
float32 confidence #confidence, can be empty
