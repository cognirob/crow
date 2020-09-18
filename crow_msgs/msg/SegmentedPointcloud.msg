# This msg wraps (segmented) pointcloud + its label
PointCloud2 pcl; 
string label; #must be one of the class labels from detector
float confidence = 0.0f; #confidence, can be empty
