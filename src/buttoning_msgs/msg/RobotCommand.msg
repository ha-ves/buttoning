# Robot arm command message
std_msgs/Header header

# Target position [x, y, z] in meters
geometry_msgs/Point position

# Target orientation (quaternion)
geometry_msgs/Quaternion orientation

# Joint angles (radians)
float64[] joint_angles

# Command type (0=move, 1=grasp, 2=release, etc.)
int32 command_type
