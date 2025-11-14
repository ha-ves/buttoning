# Quick Reference Guide

## Common Commands

### Build
```bash
# Build all packages
colcon build

# Build specific package
colcon build --packages-select buttoning_msgs
colcon build --packages-select buttoning_system

# Clean build
rm -rf build install log
colcon build
```

### Source Workspace
```bash
# Linux/Mac
source install/setup.bash

# Windows
call install\local_setup.bat
```

### Launch System
```bash
# Full system
ros2 launch buttoning_system buttoning_system.launch.py

# With custom parameters
ros2 launch buttoning_system buttoning_system.launch.py \
    left_arm_ip:=192.168.2.10 \
    right_arm_ip:=192.168.2.12 \
    model_path:=/path/to/model

# Detection only
ros2 launch buttoning_system detection_only.launch.py
```

### Run Individual Nodes
```bash
ros2 run buttoning_system detection_node
ros2 run buttoning_system hand_detection_node
ros2 run buttoning_system arm_controller_node

# With parameters
ros2 run buttoning_system detection_node --ros-args \
    -p button_threshold:=0.9 \
    -p buttonhole_threshold:=0.75
```

### Monitor Topics
```bash
# List all topics
ros2 topic list

# Monitor detection output
ros2 topic echo /detections

# Monitor hand landmarks
ros2 topic echo /hand_landmarks

# Check topic info
ros2 topic info /detections

# Monitor topic rate
ros2 topic hz /detections
```

### Visualize
```bash
# View camera image
ros2 run rqt_image_view rqt_image_view

# View all topics
rqt
```

### Parameters
```bash
# List node parameters
ros2 param list /detection_node

# Get parameter value
ros2 param get /detection_node button_threshold

# Set parameter
ros2 param set /detection_node button_threshold 0.85

# Load from YAML
ros2 run buttoning_system detection_node \
    --ros-args --params-file config/detection_params.yaml
```

### Debugging
```bash
# Node info
ros2 node info /detection_node

# View computation graph
rqt_graph

# Check node list
ros2 node list

# Run with debug logging
ros2 run buttoning_system detection_node --ros-args \
    --log-level debug
```

## Topic Reference

| Topic | Message Type | Publisher | Description |
|-------|-------------|-----------|-------------|
| `/detections` | buttoning_msgs/Detection | detection_node | Button/buttonhole detections |
| `/hand_landmarks` | buttoning_msgs/HandLandmarks | hand_detection_node | Hand landmarks |
| `/debug_image` | sensor_msgs/Image | detection_node | Visualization image |
| `/left_arm/pose` | geometry_msgs/PoseStamped | arm_controller_node | Left arm pose |
| `/right_arm/pose` | geometry_msgs/PoseStamped | arm_controller_node | Right arm pose |
| `/robot_command` | buttoning_msgs/RobotCommand | (external) | Robot commands |
| `/motion_complete` | std_msgs/Bool | arm_controller_node | Motion status |
| `/camera/image_raw` | sensor_msgs/Image | (camera driver) | Raw camera images |

## Parameter Reference

### detection_node
```yaml
max_detections: 10          # Max objects per frame
button_threshold: 0.8       # Button confidence threshold
buttonhole_threshold: 0.7   # Buttonhole confidence threshold
iou_threshold: 0.01         # Tracking IoU threshold
camera_width: 640           # Camera resolution width
camera_height: 480          # Camera resolution height
camera_fps: 30              # Camera framerate
model_path: ""              # Path to detection model
```

### arm_controller_node
```yaml
left_arm_ip: "192.168.2.10"   # Left arm IP
right_arm_ip: "192.168.2.12"  # Right arm IP
arm_username: "admin"         # Arm login username
arm_password: "admin"         # Arm login password
```

## Troubleshooting

### No topics published
```bash
# Check if nodes are running
ros2 node list

# Check node logs
ros2 run buttoning_system detection_node --ros-args --log-level debug
```

### Build errors
```bash
# Clean and rebuild
rm -rf build install log
colcon build --symlink-install

# Check dependencies
rosdep install --from-paths src --ignore-src -r -y
```

### Camera not working
```bash
# Check if RealSense is connected
rs-enumerate-devices

# Test camera manually
ros2 run realsense2_camera realsense2_camera_node
```

### Performance issues
```bash
# Check CPU usage
top

# Check topic rates
ros2 topic hz /detections

# Enable performance monitoring
ros2 run buttoning_system detection_node --ros-args --log-level debug
```

## Message Examples

### Detection Message
```yaml
header:
  stamp: {sec: 1234567890, nanosec: 123456789}
  frame_id: "camera_frame"
boxes: [100, 150, 200, 250, 300, 350, 400, 450]  # x1,y1,x2,y2 for 2 objects
classes: [0, 1]  # button, buttonhole
scores: [0.95, 0.87]
centers: [150, 200, 350, 400]  # x,y for 2 objects
track_ids: [1, 2]
```

### Hand Landmarks Message
```yaml
header:
  stamp: {sec: 1234567890, nanosec: 123456789}
  frame_id: "camera_frame"
hand_detected: true
landmarks:
  - {x: 0.5, y: 0.3, z: 0.1}  # 21 landmarks total
  - {x: 0.52, y: 0.31, z: 0.11}
  # ... (19 more)
processing_time: 0.025
```

### Robot Command Message
```yaml
header:
  stamp: {sec: 1234567890, nanosec: 123456789}
  frame_id: "base_link"
position: {x: 0.5, y: 0.2, z: 0.3}
orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}
joint_angles: [0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7]
command_type: 0  # 0=move, 1=grasp, 2=release
```

## Development Workflow

1. **Make changes** to source files
2. **Build** with `colcon build`
3. **Source** workspace with `source install/setup.bash`
4. **Test** individual node with `ros2 run`
5. **Debug** with `--ros-args --log-level debug`
6. **Launch** full system when ready
7. **Monitor** with `ros2 topic echo` and `rqt`
8. **Iterate** based on results

## Useful Aliases (add to .bashrc or .bash_aliases)

```bash
alias cb='colcon build'
alias cbs='colcon build --symlink-install'
alias cbp='colcon build --packages-select'
alias ct='colcon test'
alias sw='source install/setup.bash'
alias r2r='ros2 run'
alias r2l='ros2 launch'
alias r2t='ros2 topic'
alias r2n='ros2 node'
```

## Additional Resources

- ROS2 Documentation: https://docs.ros.org/en/humble/
- RealSense ROS2: https://github.com/IntelRealSense/realsense-ros
- cv_bridge Tutorial: https://wiki.ros.org/cv_bridge/Tutorials
- ONNX Runtime: https://onnxruntime.ai/
- Kinova ROS2: https://github.com/Kinovarobotics/ros2_kortex
