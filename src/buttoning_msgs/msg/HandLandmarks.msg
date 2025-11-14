# Hand landmarks from MediaPipe
std_msgs/Header header

# Hand detected flag
bool hand_detected

# 21 landmarks x 3 coordinates (x, y, z)
geometry_msgs/Point[] landmarks

# Processing time in seconds
float64 processing_time
