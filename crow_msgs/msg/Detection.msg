string object_type

float32 confidence

# expected to contain PointField named "pos" of FLOAT32 or FLOAT64 type, count = 3 (xyz)
# optional PointField named "normals" of FLOAT32 or FLAOT64 type, same as pos. count = 3
sensor_msgs/PointCloud2 segmented_pcl
