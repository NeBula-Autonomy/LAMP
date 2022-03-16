Header header

# Every pose has a unique key.
uint64 key
# gtsam::Key that encodes both a char and the int

# UUID for artifacts and UWB nodes where relevant
string ID

geometry_msgs/Pose pose
# (x, y, z, rotation about X axis, rotation about Y axis, rotation about Z axis)
float64[36] covariance

