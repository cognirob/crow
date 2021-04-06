# This msg wraps pose of object + its label
std_msgs/Header header
geometry_msgs/Pose[] poses
crow_msgs/PclDimensions[] size
string[] label #must be one of the class labels from detector
string[] uuid #identifying model in filter
float32 confidence #confidence, can be empty
float32[] particles #model_particles, can be empty