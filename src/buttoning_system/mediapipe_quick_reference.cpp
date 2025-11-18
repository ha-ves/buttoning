// Quick Reference: MediaPipe C++ API Cheat Sheet
// Based on your local MediaPipe installation

/* ============================================================================
   STEP 1: INITIALIZATION
   ============================================================================ */

#include "mediapipe/framework/calculator_framework.h"
#include "mediapipe/framework/formats/image_frame.h"
#include "mediapipe/framework/formats/image_frame_opencv.h"
#include "mediapipe/framework/formats/landmark.pb.h"
#include "mediapipe/framework/port/file_helpers.h"
#include "mediapipe/framework/port/parse_text_proto.h"

// Load graph config
std::string config_contents;
mediapipe::file::GetContents("path/to/graph.pbtxt", &config_contents);

// Parse config
mediapipe::CalculatorGraphConfig config =
    mediapipe::ParseTextProtoOrDie<mediapipe::CalculatorGraphConfig>(config_contents);

// Create and initialize graph
mediapipe::CalculatorGraph graph;
graph.Initialize(config);

/* ============================================================================
   STEP 2: SETUP OUTPUT CALLBACK
   ============================================================================ */

// Option A: Callback (async, recommended for ROS2)
graph.ObserveOutputStream("landmarks",
    [](const mediapipe::Packet& packet) -> absl::Status {
        auto& landmarks = packet.Get<std::vector<mediapipe::NormalizedLandmarkList>>();
        // Process landmarks (runs on MediaPipe thread)
        return absl::OkStatus();
    });

// Option B: Poller (sync)
MP_ASSIGN_OR_RETURN(mediapipe::OutputStreamPoller poller,
                    graph.AddOutputStreamPoller("landmarks"));

// Start graph (do this AFTER setting up callbacks/pollers)
graph.StartRun({});

/* ============================================================================
   STEP 3: CONVERT cv::Mat TO ImageFrame
   ============================================================================ */

cv::Mat frame_bgr;  // From OpenCV camera
cv::Mat frame_rgb;
cv::cvtColor(frame_bgr, frame_rgb, cv::COLOR_BGR2RGB);

// Create ImageFrame
auto input_frame = absl::make_unique<mediapipe::ImageFrame>(
    mediapipe::ImageFormat::SRGB,
    frame_rgb.cols,
    frame_rgb.rows,
    mediapipe::ImageFrame::kDefaultAlignmentBoundary);

// Copy pixels
cv::Mat input_frame_mat = mediapipe::formats::MatView(input_frame.get());
frame_rgb.copyTo(input_frame_mat);

/* ============================================================================
   STEP 4: SEND FRAME TO GRAPH
   ============================================================================ */

size_t timestamp_us = frame_count++;  // Monotonic timestamp

graph.AddPacketToInputStream(
    "input_video",  // Stream name from .pbtxt
    mediapipe::Adopt(input_frame.release())
        .At(mediapipe::Timestamp(timestamp_us)));

/* ============================================================================
   STEP 5: RECEIVE RESULTS
   ============================================================================ */

// With Callback: Already handled in ObserveOutputStream lambda

// With Poller:
mediapipe::Packet packet;
if (poller.Next(&packet)) {
    auto& landmarks = packet.Get<std::vector<mediapipe::NormalizedLandmarkList>>();
    // Process landmarks...
}

/* ============================================================================
   STEP 6: PROCESS HAND LANDMARKS
   ============================================================================ */

void processLandmarks(const std::vector<mediapipe::NormalizedLandmarkList>& multi_hands) {
    if (multi_hands.empty()) {
        // No hands detected
        return;
    }
    
    // Process first hand
    const auto& hand = multi_hands[0];
    
    for (int i = 0; i < hand.landmark_size(); ++i) {
        const auto& lm = hand.landmark(i);
        float x = lm.x();  // [0, 1]
        float y = lm.y();  // [0, 1]
        float z = lm.z();  // Relative depth
        
        // Convert to pixel coordinates if needed:
        int pixel_x = static_cast<int>(x * image_width);
        int pixel_y = static_cast<int>(y * image_height);
    }
}

/* ============================================================================
   STEP 7: SHUTDOWN
   ============================================================================ */

graph.CloseInputStream("input_video");
graph.WaitUntilDone();

/* ============================================================================
   HAND LANDMARK INDICES (21 total)
   ============================================================================ */

enum HandLandmark {
    WRIST = 0,
    
    THUMB_CMC = 1,
    THUMB_MCP = 2,
    THUMB_IP = 3,
    THUMB_TIP = 4,
    
    INDEX_FINGER_MCP = 5,
    INDEX_FINGER_PIP = 6,
    INDEX_FINGER_DIP = 7,
    INDEX_FINGER_TIP = 8,
    
    MIDDLE_FINGER_MCP = 9,
    MIDDLE_FINGER_PIP = 10,
    MIDDLE_FINGER_DIP = 11,
    MIDDLE_FINGER_TIP = 12,
    
    RING_FINGER_MCP = 13,
    RING_FINGER_PIP = 14,
    RING_FINGER_DIP = 15,
    RING_FINGER_TIP = 16,
    
    PINKY_MCP = 17,
    PINKY_PIP = 18,
    PINKY_DIP = 19,
    PINKY_TIP = 20
};

/* ============================================================================
   GRAPH CONFIGURATION (.pbtxt)
   ============================================================================ */

// Location: mediapipe/graphs/hand_tracking/hand_tracking_desktop_live.pbtxt

// Key streams:
// - input_stream: "input_video"    (ImageFrame)
// - output_stream: "landmarks"     (std::vector<NormalizedLandmarkList>)
// - output_stream: "handedness"    (Classification)
// - side_packet: "num_hands"       (int, default: 2)

/* ============================================================================
   ERROR HANDLING
   ============================================================================ */

auto status = graph.Initialize(config);
if (!status.ok()) {
    std::cerr << "Error: " << status.message() << std::endl;
    return;
}

// All MediaPipe functions return absl::Status
// Use MP_RETURN_IF_ERROR() macro in functions returning Status

/* ============================================================================
   THREAD SAFETY
   ============================================================================ */

// ObserveOutputStream callbacks run on MediaPipe's internal threads
// Use mutexes to protect shared data:

std::mutex landmarks_mutex_;
std::vector<mediapipe::NormalizedLandmarkList> latest_landmarks_;

graph.ObserveOutputStream("landmarks",
    [&](const mediapipe::Packet& packet) -> absl::Status {
        std::lock_guard<std::mutex> lock(landmarks_mutex_);
        latest_landmarks_ = packet.Get<std::vector<mediapipe::NormalizedLandmarkList>>();
        return absl::OkStatus();
    });

/* ============================================================================
   COMMON ISSUES
   ============================================================================ */

// 1. BGR vs RGB: OpenCV uses BGR, MediaPipe expects RGB
//    Always convert: cv::cvtColor(frame, frame_rgb, cv::COLOR_BGR2RGB);

// 2. Timestamps must be monotonic (increasing)
//    Use frame counter, not wall clock time

// 3. Graph must be started AFTER ObserveOutputStream
//    Wrong: StartRun() -> ObserveOutputStream()
//    Right: ObserveOutputStream() -> StartRun()

// 4. Packets are immutable - use Adopt() to transfer ownership
//    mediapipe::Adopt(input_frame.release())

// 5. MatView doesn't copy - use copyTo()
//    cv::Mat mat = mediapipe::formats::MatView(frame.get());
//    source.copyTo(mat);  // ✓ Correct
//    mat = source;        // ✗ Wrong (shallow copy)
