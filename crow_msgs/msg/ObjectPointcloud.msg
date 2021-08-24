# This msg wraps object pointcloud + its uuid
std_msgs/Header header
sensor_msgs/PointCloud2[] pcl
string[] uuid # given by filter
string[] labels # class
