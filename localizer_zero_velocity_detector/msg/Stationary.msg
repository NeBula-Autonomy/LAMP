std_msgs/Header header
int8 status

# Only valid if stationary: defaults to NaN
geometry_msgs/Vector3 average_acceleration

int8 INITIALIZING   = -1
int8 STATIONARY     = 0
int8 NONSTATIONARY  = 1
