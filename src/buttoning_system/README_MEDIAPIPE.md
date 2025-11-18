# MediaPipe C++ Integration for ROS2 Hand Detection

This directory contains three implementations to help you understand and use MediaPipe C++ API:

## Files Overview

### 1. **Standalone Test** (`src/mediapipe_hand_test.cpp`)
- Pure MediaPipe C++ application (no ROS2 dependencies)
- Reads from webcam and displays hand landmarks in real-time
- Perfect for testing MediaPipe before integrating with ROS2
- **Build with Bazel** (MediaPipe's native build system)

### 2. **ROS2 Integration** (`hand_detection_node.cpp` + `hand_detection_node.hpp`)
- Full MediaPipe integration into ROS2 node
- Subscribes to camera images and publishes hand landmarks
- Uses background thread for processing
- Thread-safe landmark callbacks

### 3. **Build Configuration** (`CMakeLists.txt` + `BUILD`)
- CMake configuration for ROS2 workspace
- Bazel BUILD file for standalone test

---

## Quick Start Guide

### Option A: Test MediaPipe Standalone (Recommended First Step)

This approach tests MediaPipe independently before dealing with ROS2 complexity.

#### Prerequisites
```powershell
# Install Bazelisk (easiest way to get correct Bazel version)
choco install bazelisk

# Or download from: https://github.com/bazelbuild/bazelisk/releases
# Place bazelisk.exe in your PATH as bazel.exe
```

#### Build and Run
```powershell
# Navigate to MediaPipe directory
cd c:\Users\HaveS\source\Ros2ws\src\mediapipe_pkg\mediapipe

# Build the standalone test
bazel build //buttoning_system:mediapipe_hand_test --define MEDIAPIPE_DISABLE_GPU=1

# Run with webcam
bazel-bin\buttoning_system\mediapipe_hand_test.exe `
  --graph_config=mediapipe/graphs/hand_tracking/hand_tracking_desktop_live.pbtxt `
  --camera_id=0 `
  --max_frames=0

# Press 'q' or ESC to exit
```

**What to Expect:**
- Window opens showing webcam feed
- Green skeleton drawn on detected hands
- Console shows frame count and landmark detection stats
- Red dots on fingertips, blue dots on joints

---

### Option B: Build with ROS2 (Advanced)

MediaPipe doesn't integrate easily with CMake. Two approaches:

#### Approach 1: Build MediaPipe as Libraries (Complex)

```powershell
# Build MediaPipe framework libraries
cd c:\Users\HaveS\source\Ros2ws\src\mediapipe_pkg\mediapipe

bazel build //mediapipe/framework:calculator_framework `
             //mediapipe/framework/formats:image_frame `
             //mediapipe/framework/formats:image_frame_opencv `
             --define MEDIAPIPE_DISABLE_GPU=1

# Then manually link these in CMakeLists.txt
# (This is tricky - see CMakeLists.txt comments)
```

#### Approach 2: Conditional Compilation (Easier)

The code is already set up for this:

```powershell
# Build without MediaPipe (placeholders only)
cd c:\Users\HaveS\source\Ros2ws
colcon build --packages-select buttoning_system --cmake-args -DBUILD_WITH_MEDIAPIPE=OFF

# Build with MediaPipe (requires libraries from Approach 1)
colcon build --packages-select buttoning_system --cmake-args -DBUILD_WITH_MEDIAPIPE=ON
```

---

## Understanding the Code

### Key Concepts from `demo_run_graph_main.cc`

1. **Graph Initialization**
   ```cpp
   mediapipe::CalculatorGraph graph;
   graph.Initialize(config);
   graph.StartRun({});
   ```

2. **Frame Conversion** (BGR → RGB → ImageFrame)
   ```cpp
   cv::Mat frame_rgb;
   cv::cvtColor(frame_bgr, frame_rgb, cv::COLOR_BGR2RGB);
   
   auto input_frame = absl::make_unique<mediapipe::ImageFrame>(
       mediapipe::ImageFormat::SRGB, frame_rgb.cols, frame_rgb.rows,
       mediapipe::ImageFrame::kDefaultAlignmentBoundary);
   
   cv::Mat input_mat = mediapipe::formats::MatView(input_frame.get());
   frame_rgb.copyTo(input_mat);
   ```

3. **Sending to Graph**
   ```cpp
   graph.AddPacketToInputStream(
       "input_video",
       mediapipe::Adopt(input_frame.release())
           .At(mediapipe::Timestamp(frame_count)));
   ```

4. **Receiving Landmarks** (Callback)
   ```cpp
   graph.ObserveOutputStream("landmarks",
       [](const mediapipe::Packet& packet) -> absl::Status {
           auto& landmarks = 
               packet.Get<std::vector<mediapipe::NormalizedLandmarkList>>();
           // Process landmarks...
           return absl::OkStatus();
       });
   ```

### Hand Landmark Format

From `mediapipe/framework/formats/landmark.proto`:
```protobuf
message NormalizedLandmark {
  float x;  // [0, 1] normalized to image width
  float y;  // [0, 1] normalized to image height
  float z;  // Relative depth
  float visibility;  // Optional
  float presence;    // Optional
}

message NormalizedLandmarkList {
  repeated NormalizedLandmark landmark = 1;  // 21 landmarks per hand
}
```

**MediaPipe Hand Topology (21 landmarks):**
```
       8   12  16  20   (fingertips)
       |   |   |   |
       7   11  15  19
       |   |   |   |
       6   10  14  18
       |   |   |   |
    4  5---9---13--17
    |  |
    3  |
    |  |
    2  |
    |  |
    1  |
    |  |
    0 (wrist)
```

Landmarks:
- 0: Wrist
- 1-4: Thumb
- 5-8: Index finger
- 9-12: Middle finger
- 13-16: Ring finger
- 17-20: Pinky

---

## Troubleshooting

### Error: "Failed to load graph config"
**Solution:** Check that `graph_config_path_` points to valid .pbtxt file
```cpp
// In hand_detection_node.cpp, line ~150
graph_config_path_ = "c:/Users/HaveS/source/Ros2ws/src/mediapipe_pkg/mediapipe/"
                    "mediapipe/graphs/hand_tracking/hand_tracking_desktop_live.pbtxt";
```

### Error: "Failed to initialize MediaPipe graph"
**Causes:**
- Missing model files (should be in `mediapipe/modules/hand_landmark/`)
- GPU enabled but no GPU available (use `--define MEDIAPIPE_DISABLE_GPU=1`)

### Bazel Build Errors
```powershell
# Clean and rebuild
bazel clean
bazel build //buttoning_system:mediapipe_hand_test --define MEDIAPIPE_DISABLE_GPU=1
```

### CMake Can't Find MediaPipe
This is expected! MediaPipe doesn't provide CMake config files. Options:
1. Use Bazel for MediaPipe parts (standalone test)
2. Build MediaPipe separately and manually link
3. Set `BUILD_WITH_MEDIAPIPE=OFF` and use ROS2 with placeholder code

---

## Performance Tips

- **CPU vs GPU:** CPU is easier to build on Windows. GPU requires CUDA/DirectX.
- **Resolution:** Lower camera resolution (640x480) for faster processing
- **Threading:** MediaPipe graph runs in its own threads; callbacks are async
- **Timestamps:** Use monotonic frame count, not wall clock time

---

## Next Steps

1. **Test standalone:** Build and run `mediapipe_hand_test.cpp`
2. **Study the code:** Read comments in all three files
3. **Understand callbacks:** Note how landmarks are thread-safe in ROS2 node
4. **Try modifications:** 
   - Change max hands detected (edit .pbtxt `num_hands` value)
   - Draw landmarks in `hand_detection_node.cpp`
   - Add gesture recognition logic

---

## Additional Resources

- **MediaPipe Docs:** https://developers.google.com/mediapipe
- **Hand Tracking Guide:** https://developers.google.com/mediapipe/solutions/vision/hand_landmarker
- **Bazel Tutorial:** https://bazel.build/start
- **Graph Configs:** `mediapipe/graphs/hand_tracking/`
- **Example Code:** `mediapipe/examples/desktop/`

---

## File Locations

```
buttoning_system/
├── BUILD                           # Bazel build config
├── README_MEDIAPIPE.md            # This file
├── CMakeLists.txt                 # ROS2 build config (updated)
├── include/
│   └── buttoning_system/
│       └── hand_detection_node.hpp  # Header with MediaPipe integration
└── src/
    ├── hand_detection_node.cpp      # ROS2 node implementation
    └── mediapipe_hand_test.cpp      # Standalone test program
```

**MediaPipe Installation:**
```
c:/Users/HaveS/source/Ros2ws/src/mediapipe_pkg/mediapipe/
├── mediapipe/
│   ├── framework/                   # Core MediaPipe framework
│   ├── graphs/hand_tracking/        # Hand tracking configurations
│   ├── modules/hand_landmark/       # Hand landmark detection
│   └── examples/desktop/            # Example applications
└── BUILD files, WORKSPACE, etc.
```

---

**Good luck! Start with the standalone test first. 🚀**
