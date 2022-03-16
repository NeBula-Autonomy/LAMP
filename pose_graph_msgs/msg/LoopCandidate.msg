Header header

uint64 key_from
uint64 key_to

# Odometric (initial) pose of the nodes 
geometry_msgs/Pose pose_from
geometry_msgs/Pose pose_to

int32 type 

# Type enums
int32 PROXIMITY         = 0
int32 MANUAL            = 1
int32 JUNCTION          = 2
int32 VISUAL            = 3

float64 value