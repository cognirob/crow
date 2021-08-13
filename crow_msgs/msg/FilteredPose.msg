# This msg wraps pose of object + its label
std_msgs/Header header
geometry_msgs/Pose[] poses
crow_msgs/PclDimensions[] size
bool[] tracked
string[] label #must be one of the class labels from detector
string[] uuid #identifying model in filter
float32 confidence #confidence, can be empty
#crow_msgs/Particles[] particles #model_particles, can be empty
std_msgs/Float32MultiArray[] particles
