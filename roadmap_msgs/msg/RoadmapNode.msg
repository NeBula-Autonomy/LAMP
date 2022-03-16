time stamp

# Unique index of the node
uint32 index

# Pose of the node and its covariance
geometry_msgs/PoseWithCovariance pose

# Type of the node
uint8 type
uint8 TYPE_DEFAULT = 0
uint8 TYPE_BASE = 1
uint8 TYPE_FRONTIER = 2
uint8 TYPE_TERMINAL = 3
uint8 TYPE_COMMS_RADIO = 4
uint8 TYPE_UWB_RADIO = 5
uint8 TYPE_STAIR_ENTRY = 6
uint8 TYPE_ARTIFACT = 7
uint8 TYPE_COMMS_CHECKPOINT = 8

# Optional properties
RoadmapProperty[] properties
