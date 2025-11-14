# SORT Tracker - Simple Online and Realtime Tracking

## Overview

This is a C++ implementation of the SORT (Simple Online and Realtime Tracking) algorithm for multi-object tracking in computer vision applications. SORT combines Kalman filtering for motion prediction with the Hungarian algorithm for data association.

**Paper:** [Simple Online and Realtime Tracking](https://arxiv.org/abs/1602.00763) by Alex Bewley et al., 2016

## Features

- **Kalman Filter-based Motion Prediction**: Uses a constant velocity model to predict object positions
- **Hungarian Algorithm**: Optimal assignment of detections to existing tracks based on IOU (Intersection over Union)
- **Track Lifecycle Management**: Automatic creation, confirmation, and deletion of tracks
- **Configurable Parameters**: Adjustable thresholds for tracking behavior
- **Header-only Integration**: Easy to integrate into ROS2 nodes

## Components

### 1. KalmanBoxTracker

Tracks individual bounding boxes using a 7-dimensional state vector:
- **State**: `[x, y, s, r, vx, vy, vs]`
  - `x, y`: Center position
  - `s`: Scale (area = width × height)
  - `r`: Aspect ratio (width / height)
  - `vx, vy, vs`: Velocities

**Key Methods:**
- `predict()`: Predicts the next state using motion model
- `update(bbox)`: Updates state with new detection
- `getState()`: Returns current bounding box estimate

### 2. SortTracker

Manages multiple tracks and performs data association.

**Key Methods:**
- `update(detections, scores)`: Main tracking function
- `getTrackIds()`: Returns track IDs for current detections
- `reset()`: Clears all tracks
- `getNumTracks()`: Returns number of active tracks

## Parameters

| Parameter | Default | Description |
|-----------|---------|-------------|
| `max_age` | 1 | Maximum frames to keep a track alive without detections |
| `min_hits` | 3 | Minimum consecutive detections before track is confirmed |
| `iou_threshold` | 0.3 | Minimum IOU for matching detections to tracks |

## Usage

### Basic Example

```cpp
#include "buttoning_system/sort_tracker.hpp"

// Create tracker
SortTracker tracker(
    3,    // max_age: keep tracks for 3 frames without detection
    3,    // min_hits: require 3 consecutive hits to confirm
    0.3   // iou_threshold: minimum 0.3 IOU for matching
);

// Each frame:
std::vector<cv::Rect> detections = {
    cv::Rect(100, 100, 50, 50),
    cv::Rect(200, 200, 60, 60)
};
std::vector<float> scores = {0.9f, 0.85f};

// Update tracker
auto tracked_objects = tracker.update(detections, scores);

// Get track IDs for detections
std::vector<int> track_ids = tracker.getTrackIds();

// Process tracked objects
for (const auto& [track_id, bbox] : tracked_objects) {
    std::cout << "Track " << track_id << ": "
              << bbox.x << ", " << bbox.y << std::endl;
}
```

### Integration with Detection Node

The SORT tracker is integrated into the `DetectionNode` class:

```cpp
// In detection_node.hpp
#include "buttoning_system/sort_tracker.hpp"

class DetectionNode : public rclcpp::Node {
private:
    std::unique_ptr<SortTracker> tracker_;
    // ...
};

// In detection_node.cpp (constructor)
tracker_ = std::make_unique<SortTracker>(max_age_, min_hits_, iou_threshold_);

// In runTracking()
auto tracked_objects = tracker_->update(pred_boxes_, pred_scores_);
track_ids_ = tracker_->getTrackIds();
```

## Algorithm Flow

1. **Prediction**: All existing tracks predict their new positions using Kalman filter
2. **Association**: Match detections to predictions using IOU-based Hungarian algorithm
3. **Update**: Update matched tracks with new detections
4. **Create**: Create new tracks for unmatched detections
5. **Delete**: Remove tracks that haven't been matched for `max_age` frames
6. **Return**: Return only confirmed tracks (hits >= `min_hits`)

## Configuration for Different Scenarios

### High Frame Rate, Stable Objects (e.g., buttons/buttonholes)
```cpp
SortTracker tracker(
    10,   // max_age: objects rarely disappear
    1,    // min_hits: confirm quickly
    0.01  // iou_threshold: very low, objects don't overlap
);
```

### Low Frame Rate, Fast Moving Objects
```cpp
SortTracker tracker(
    2,    // max_age: short memory
    5,    // min_hits: reduce false positives
    0.5   // iou_threshold: higher threshold
);
```

### Crowded Scenes with Occlusions
```cpp
SortTracker tracker(
    5,    // max_age: maintain tracks during occlusion
    3,    // min_hits: balanced
    0.3   // iou_threshold: standard value
);
```

## Performance Considerations

- **Computational Complexity**: O(n³) for Hungarian algorithm, where n = min(detections, tracks)
- **Real-time Performance**: Suitable for up to ~100 objects at 30 FPS
- **Memory**: Minimal - only active tracks stored
- **Accuracy**: Trade-off between `min_hits` (fewer false positives) and tracking latency

## Dependencies

- **OpenCV**: For bounding box representation (`cv::Rect`)
- **Eigen3**: For matrix operations in Kalman filter
- **C++17**: For modern C++ features

## Build Instructions

The SORT tracker is automatically built with the `buttoning_system` package:

```bash
# Build the workspace
cd ~/Ros2ws
colcon build --packages-select buttoning_system

# Run example (optional)
./build/buttoning_system/sort_tracker_example
```

## Tuning Tips

1. **`max_age`**: 
   - Increase if objects frequently disappear temporarily
   - Decrease for faster track cleanup
   - Typical range: 1-30 frames

2. **`min_hits`**:
   - Increase to reduce false positives
   - Decrease for faster track confirmation
   - Typical range: 1-10 hits

3. **`iou_threshold`**:
   - Increase if objects never overlap
   - Decrease for crowded scenes
   - Typical range: 0.1-0.5

## Limitations

- No appearance model (only motion-based)
- No re-identification after track loss
- Simple constant velocity motion model
- Greedy Hungarian algorithm (not optimal for very large problems)

## Future Enhancements

- [ ] Deep SORT: Add appearance features using deep learning
- [ ] Improved motion models (e.g., constant acceleration)
- [ ] Track re-identification
- [ ] Multi-class tracking support
- [ ] Visualization utilities

## References

- Bewley, A., Ge, Z., Ott, L., Ramos, F., & Upcroft, B. (2016). Simple online and realtime tracking. In 2016 IEEE International Conference on Image Processing (ICIP).
- Kalman filter: https://en.wikipedia.org/wiki/Kalman_filter
- Hungarian algorithm: https://en.wikipedia.org/wiki/Hungarian_algorithm

## License

This implementation is part of the buttoning_system ROS2 package.
