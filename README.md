# Buttoning Dual-Arm System - ROS2 C++ Implementation

This package contains ROS2 C++ nodes for a dual-arm buttoning system, converted from the Python Jupyter notebook implementation.

## Package Structure

```
ros2_ws/
├── src/
│   ├── buttoning_msgs/          # Custom message definitions
│   │   ├── msg/
│   │   │   ├── Detection.msg        # Object detection results
│   │   │   ├── HandLandmarks.msg    # Hand detection results
│   │   │   └── RobotCommand.msg     # Robot arm commands
│   │   ├── CMakeLists.txt
│   │   └── package.xml
│   │
│   └── buttoning_system/        # Main system nodes
│       ├── include/buttoning_system/
│       │   ├── detection_node.hpp          # Main detection loop
│       │   ├── hand_detection_node.hpp     # Hand detection helper thread
│       │   └── arm_controller_node.hpp     # Dual-arm controller
│       ├── src/
│       │   ├── detection_node.cpp
│       │   ├── hand_detection_node.cpp
│       │   └── arm_controller_node.cpp
│       ├── launch/
│       │   ├── buttoning_system.launch.py  # Launch all nodes
│       │   └── detection_only.launch.py    # Launch detection only
│       ├── config/
│       │   ├── detection_params.yaml
│       │   └── arm_params.yaml
│       ├── CMakeLists.txt
│       └── package.xml
```

## Nodes

### 1. Detection Node (Main Loop)
**Executable:** `detection_node`

Corresponds to the main loop in the Python notebook (cell around line 2845).

**Functionality:**
- Captures frames from RealSense camera
- Runs object detection (button/buttonhole) using AI model
- Applies per-class confidence thresholding
- Performs object tracking (SORT algorithm)
- Publishes detection results

**Topics:**
- **Publishes:**
  - `/detections` (buttoning_msgs/Detection) - Detected objects with bounding boxes, classes, scores
  - `/debug_image` (sensor_msgs/Image) - Visualization image
  
- **Subscribes:**
  - `/hand_landmarks` (buttoning_msgs/HandLandmarks) - Hand detection from helper node

**Parameters:**
- `max_detections`: Maximum objects per frame (default: 10)
- `button_threshold`: Confidence threshold for buttons (default: 0.8)
- `buttonhole_threshold`: Confidence threshold for buttonholes (default: 0.7)
- `iou_threshold`: IoU threshold for tracking (default: 0.01)
- `model_path`: Path to detection model

### 2. Hand Detection Node (Helper Thread)
**Executable:** `hand_detection_node`

Corresponds to the `detect_hands()` thread helper in Python notebook (cell around line 1087).

**Functionality:**
- Runs MediaPipe hand detection in separate thread
- Processes frames asynchronously from main detection loop
- Publishes hand landmarks with processing time

**Topics:**
- **Publishes:**
  - `/hand_landmarks` (buttoning_msgs/HandLandmarks) - Hand detection results
  
- **Subscribes:**
  - `/camera/image_raw` (sensor_msgs/Image) - Input images

**Threading:**
Uses C++ `std::thread` with condition variables for efficient asynchronous processing, mimicking the Python threading model.

### 3. Arm Controller Node
**Executable:** `arm_controller_node`

Controls dual Kinova arms for buttoning tasks.

**Functionality:**
- Manages connections to left and right Kinova arms
- Receives detection results and plans motions
- Executes dual-arm coordination
- Handles grasp/release commands

**Topics:**
- **Publishes:**
  - `/left_arm/pose` (geometry_msgs/PoseStamped) - Left arm pose
  - `/right_arm/pose` (geometry_msgs/PoseStamped) - Right arm pose
  - `/motion_complete` (std_msgs/Bool) - Motion completion flag
  
- **Subscribes:**
  - `/detections` (buttoning_msgs/Detection) - Object detections
  - `/robot_command` (buttoning_msgs/RobotCommand) - Motion commands

**Parameters:**
- `left_arm_ip`: IP address of left arm (default: "192.168.2.10")
- `right_arm_ip`: IP address of right arm (default: "192.168.2.12")
- `arm_username`: Arm login username (default: "admin")
- `arm_password`: Arm login password (default: "admin")

## Building

### Prerequisites

Install ROS2 (tested with Humble/Iron) and dependencies:

```bash
# Install OpenCV
sudo apt install libopencv-dev

# Install cv_bridge
sudo apt install ros-$ROS_DISTRO-cv-bridge ros-$ROS_DISTRO-image-transport

# Optional: Install RealSense SDK
# sudo apt install ros-$ROS_DISTRO-realsense2-camera librealsense2-dev

# Optional: Install ONNX Runtime or TensorRT for inference
# Follow platform-specific installation guides
```

### Build Instructions

```bash
cd ros2_ws
colcon build --packages-select buttoning_msgs buttoning_system
source install/setup.bash
```

## Running

### Launch Full System

```bash
ros2 launch buttoning_system buttoning_system.launch.py
```

### Launch Detection Only (for testing)

```bash
ros2 launch buttoning_system detection_only.launch.py
```

### Run Individual Nodes

```bash
# Terminal 1 - Hand detection helper
ros2 run buttoning_system hand_detection_node

# Terminal 2 - Main detection
ros2 run buttoning_system detection_node

# Terminal 3 - Arm controller
ros2 run buttoning_system arm_controller_node
```

## Placeholders and TODOs

The current implementation includes **placeholders** for libraries that may not be available or require platform-specific setup:

### 1. RealSense Camera Integration
```cpp
// TODO: Replace with actual librealsense2 implementation
// #include <librealsense2/rs.hpp>
// rs2::pipeline pipeline;
```

### 2. AI Model Inference
```cpp
// TODO: Use ONNX Runtime or TensorRT
// #include <onnxruntime_cxx_api.h>
// Ort::Session onnx_session;
```

### 3. MediaPipe Hand Detection
```cpp
// TODO: Use MediaPipe C++ API or Python bindings via pybind11
// #include <mediapipe/framework/calculator_framework.h>
```

### 4. SORT Tracker
```cpp
// TODO: Implement SORT tracking algorithm
// std::unique_ptr<SortTracker> tracker_;
```

### 5. Kinova Kortex API
```cpp
// TODO: Link against kortex_api
// #include <kortex_api/BaseClient.h>
// std::unique_ptr<Kinova::Api::BaseClient> arm_base_;
```

## Integration Steps

To make this fully functional:

1. **Camera Integration**: Uncomment RealSense code and link against `librealsense2`
2. **Model Inference**: Add ONNX Runtime or TensorRT, load detection model
3. **Hand Detection**: Integrate MediaPipe C++ API or use ROS bridge to Python node
4. **Tracking**: Implement SORT algorithm or use existing library
5. **Robot Control**: Link Kinova kortex_api and implement motion commands

## Architecture

The system follows ROS2 best practices:

- **Modularity**: Separate nodes for detection, hand tracking, and control
- **Threading**: Background processing in hand detection node
- **Message Passing**: Standard ROS2 pub/sub for inter-node communication
- **Parameters**: Configurable via YAML files and launch arguments
- **Lifecycle**: Proper resource management in constructors/destructors

## Performance Notes

From the Python notebook, typical timing:
- RealSense frame capture: ~0.03s
- Object detection inference: ~0.05-0.1s
- Hand detection (MediaPipe): ~0.02-0.05s
- Tracking: <0.01s

Target loop rate: ~30Hz for real-time operation.

## License

Apache-2.0

## Contact

For questions or issues, contact [maintainer email].
