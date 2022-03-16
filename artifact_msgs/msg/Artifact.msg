# Header and Timestamp
Header header

# Robot name
string name

# Artifact ID
string id

# Artifact Parent ID
# If not a repeat observation, same as id
string parent_id

# Point
geometry_msgs/PointStamped point

# Localization covariance matrix
float32[9] covariance

# Classification confidence level
float32 confidence
float32 confidence_base  # Unused

# Classification
string label

# Thumbnail
sensor_msgs/CompressedImage thumbnail

# boundix box for each thumbnail


# Detection modality
string detection_source

# bounding box
# 0. best confidence
# 1. most bright
# 2. less blurry
# 3. closest detection
# 4. last detection
int16[5] xmin
int16[5] ymin
int16[5] xmax
int16[5] ymax

# Scorability
float32 scorability
artifact_msgs/ScorabilityMetrics scorability_metrics

# Detection statistics
int16 num_observations

# Robot positions where the observations were made
geometry_msgs/Point[] observation_points
