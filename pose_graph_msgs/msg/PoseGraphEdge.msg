uint64 key_from
uint64 key_to

int32 type 

# Type enums
int32 ODOM              = 0
int32 PRIOR             = 1
int32 IMU               = 2
int32 LOOPCLOSE         = 3
int32 ARTIFACT          = 4
int32 UWB_RANGE         = 5 
int32 UWB_BETWEEN       = 6
int32 INTERROB_RANGE    = 7
int32 INTERROB_BETW     = 8



# Pose transformation encoded in the edge
geometry_msgs/Pose pose
# (x, y, z, rotation about X axis, rotation about Y axis, rotation about Z axis)
float64[36] covariance

float64 range
float64 range_error