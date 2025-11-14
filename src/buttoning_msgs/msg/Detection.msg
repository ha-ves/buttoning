# Detection message for button/buttonhole instances
std_msgs/Header header

# Bounding boxes [x1, y1, x2, y2]
int32[] boxes

# Class IDs (0=button, 1=buttonhole)
int32[] classes

# Confidence scores
float32[] scores

# Center points [x, y]
int32[] centers

# Tracking IDs
int32[] track_ids
