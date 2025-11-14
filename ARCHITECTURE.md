# ROS2 Node Architecture for Buttoning System

```
┌─────────────────────────────────────────────────────────────────────┐
│                        Buttoning Dual-Arm System                     │
└─────────────────────────────────────────────────────────────────────┘

┌──────────────────────┐
│  RealSense Camera    │  (Hardware)
│   L515 Depth RGB     │
└──────────┬───────────┘
           │ USB
           ▼
┌──────────────────────────────────────────────────────────────────────┐
│                         DETECTION NODE (Main Loop)                    │
│  Corresponds to: Cell #53 (line 2845) - main while loop              │
├──────────────────────────────────────────────────────────────────────┤
│  • Capture frames from RealSense                                      │
│  • Preprocess (crop to valid region)                                 │
│  • Run Detectron2/ONNX inference                                     │
│  • Apply per-class thresholding                                      │
│  • Track objects (SORT algorithm)                                    │
│  • Publish detection results                                         │
└────────┬──────────────────────────┬────────────────────────────┬─────┘
         │                          │                            │
         │ Subscribes               │ Publishes                  │
         ▼                          ▼                            │
┌─────────────────────┐   ┌──────────────────────┐             │
│ /hand_landmarks     │   │  /detections         │             │
│ (from hand node)    │   │  /debug_image        │             │
└─────────────────────┘   └──────────────────────┘             │
                                                                 │
         ┌───────────────────────────────────────────────────────┘
         │ Image data shared
         ▼
┌──────────────────────────────────────────────────────────────────────┐
│               HAND DETECTION NODE (Helper Thread)                     │
│  Corresponds to: Cell #29 (line 1087) - detect_hands() function      │
├──────────────────────────────────────────────────────────────────────┤
│  • Runs in separate thread (std::thread)                             │
│  • Waits for new frame via condition_variable                        │
│  • Process with MediaPipe hand detector                              │
│  • Extract 21 hand landmarks                                         │
│  • Publish hand landmarks + processing time                          │
└────────┬─────────────────────────────────────────────────────────────┘
         │ Publishes
         ▼
┌─────────────────────┐
│ /hand_landmarks     │
└─────────────────────┘


┌──────────────────────────────────────────────────────────────────────┐
│                    ARM CONTROLLER NODE                                │
│  Corresponds to: Arm control code in notebook                        │
├──────────────────────────────────────────────────────────────────────┤
│  • Connect to left & right Kinova arms (TCP)                         │
│  • Subscribe to detections                                           │
│  • Plan dual-arm motion                                              │
│  • Execute grasp/release/move commands                               │
│  • Coordinate button + buttonhole alignment                          │
└────────┬──────────────────────────┬────────────────────────────┬─────┘
         │ Subscribes               │ Publishes                  │
         ▼                          ▼                            ▼
┌─────────────────────┐   ┌──────────────────────┐   ┌──────────────────┐
│ /detections         │   │ /left_arm/pose       │   │ /right_arm/pose  │
│ /robot_command      │   │ /motion_complete     │   │                  │
└─────────────────────┘   └──────────────────────┘   └──────────────────┘
                                     │
                                     ▼
                          ┌──────────────────────┐
                          │  Kinova Arms         │
                          │  Left: 192.168.2.10  │
                          │  Right: 192.168.2.12 │
                          └──────────────────────┘


═══════════════════════════════════════════════════════════════════════

MESSAGE TYPES (buttoning_msgs package):

┌────────────────────────────────────────────────────────────────┐
│ Detection.msg                                                   │
├────────────────────────────────────────────────────────────────┤
│ • header (timestamp, frame_id)                                 │
│ • boxes[] - Bounding boxes [x1,y1,x2,y2] for each object       │
│ • classes[] - Class IDs (0=button, 1=buttonhole)               │
│ • scores[] - Confidence scores                                 │
│ • centers[] - Center points [x,y]                              │
│ • track_ids[] - Tracking IDs from SORT                         │
└────────────────────────────────────────────────────────────────┘

┌────────────────────────────────────────────────────────────────┐
│ HandLandmarks.msg                                               │
├────────────────────────────────────────────────────────────────┤
│ • header                                                        │
│ • hand_detected (bool)                                         │
│ • landmarks[] - 21 x (x,y,z) points                            │
│ • processing_time (float64)                                    │
└────────────────────────────────────────────────────────────────┘

┌────────────────────────────────────────────────────────────────┐
│ RobotCommand.msg                                                │
├────────────────────────────────────────────────────────────────┤
│ • header                                                        │
│ • position (Point - x,y,z)                                     │
│ • orientation (Quaternion)                                     │
│ • joint_angles[] (float64)                                     │
│ • command_type (0=move, 1=grasp, 2=release)                    │
└────────────────────────────────────────────────────────────────┘


═══════════════════════════════════════════════════════════════════════

THREADING MODEL:

Python Notebook Threading:
┌─────────────────────────────┐
│ Main Thread                 │  ← Main while loop
│ - Camera capture            │
│ - Object detection          │
│ - Tracking                  │
│ - hand_event.set()          │  ← Signal helper
│ - hand_finish.wait()        │  ← Wait for helper
└─────────────────────────────┘
              ║
              ║ threading.Event
              ║
┌─────────────────────────────┐
│ Helper Thread               │  ← detect_hands()
│ - hand_event.wait()         │  ← Wait for signal
│ - MediaPipe processing      │
│ - hand_finish.set()         │  ← Signal completion
└─────────────────────────────┘

ROS2 C++ Implementation:
┌─────────────────────────────┐
│ detection_node              │  ← Timer callback (main loop)
│ - Camera capture            │
│ - Object detection          │
│ - Tracking                  │
│ - Publishes detection       │
│ - Subscribes hand_landmarks │
└─────────────────────────────┘
              ║
              ║ ROS2 pub/sub
              ║
┌─────────────────────────────┐
│ hand_detection_node         │  ← Separate node
│ std::thread worker:         │
│ - condition_variable wait   │
│ - MediaPipe processing      │
│ - Publishes hand_landmarks  │
└─────────────────────────────┘


═══════════════════════════════════════════════════════════════════════

DATA FLOW TIMING (from notebook):

Frame N:
  t=0.000: Camera capture starts
  t=0.030: Frame received (rs2_time)
  t=0.035: Preprocessing done (preproc_time)
  t=0.036: Trigger hand detection (hand_event.set())
  t=0.036: Start inference
  t=0.086: Inference done (rcnn_time ~0.05s)
  t=0.087: Wait for hand detection (hand_finish.wait())
  t=0.106: Hand detection done (~0.02s in parallel)
  t=0.107: Tracking + publish
  t=0.110: Loop to next frame

Target: ~30 Hz (0.033s per frame)

