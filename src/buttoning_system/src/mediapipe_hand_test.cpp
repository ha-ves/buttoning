// Standalone MediaPipe Hand Tracking Test
// Build and test MediaPipe independently before ROS2 integration
//
// Build with Bazel:
//   bazel build //buttoning_system:mediapipe_hand_test
//
// Or compile manually:
//   g++ -std=c++17 mediapipe_hand_test.cpp -o mediapipe_hand_test \
//       -I/path/to/mediapipe -L/path/to/mediapipe/bazel-bin \
//       -lmediapipe_framework -labsl_log -lopencv_core -lopencv_imgproc \
//       -lopencv_highgui -lopencv_videoio

#include <iostream>
#include <memory>
#include <opencv2/opencv.hpp>

#include "absl/flags/flag.h"
#include "absl/flags/parse.h"
#include "absl/log/absl_log.h"
#include "mediapipe/framework/calculator_framework.h"
#include "mediapipe/framework/formats/image_frame.h"
#include "mediapipe/framework/formats/image_frame_opencv.h"
#include "mediapipe/framework/formats/landmark.pb.h"
#include "mediapipe/framework/port/file_helpers.h"
#include "mediapipe/framework/port/parse_text_proto.h"
#include "mediapipe/framework/port/status.h"

ABSL_FLAG(std::string, graph_config, 
          "c:/Users/HaveS/source/Ros2ws/src/mediapipe_pkg/mediapipe/mediapipe/graphs/hand_tracking/hand_tracking_desktop_live.pbtxt",
          "Path to hand tracking graph config file");
ABSL_FLAG(int, camera_id, 0, "Camera device ID");
ABSL_FLAG(int, max_frames, 300, "Maximum frames to process (0 = unlimited)");

constexpr char kInputStream[] = "input_video";
constexpr char kLandmarksStream[] = "landmarks";
constexpr char kWindowName[] = "MediaPipe Hand Tracking Test";

class HandTrackingTest {
public:
    HandTrackingTest() : frame_count_(0), landmarks_received_(0) {}
    
    absl::Status Initialize() {
        // Load graph configuration
        std::string config_contents;
        MP_RETURN_IF_ERROR(mediapipe::file::GetContents(
            absl::GetFlag(FLAGS_graph_config), &config_contents));
        
        ABSL_LOG(INFO) << "Loaded graph config from: " << absl::GetFlag(FLAGS_graph_config);
        
        // Parse configuration
        mediapipe::CalculatorGraphConfig config =
            mediapipe::ParseTextProtoOrDie<mediapipe::CalculatorGraphConfig>(
                config_contents);
        
        // Initialize graph
        graph_ = std::make_unique<mediapipe::CalculatorGraph>();
        MP_RETURN_IF_ERROR(graph_->Initialize(config));
        
        // Register callback for landmarks output
        MP_RETURN_IF_ERROR(graph_->ObserveOutputStream(
            kLandmarksStream,
            [this](const mediapipe::Packet& packet) -> absl::Status {
                return ProcessLandmarks(packet);
            }));
        
        // Start the graph
        MP_RETURN_IF_ERROR(graph_->StartRun({}));
        
        ABSL_LOG(INFO) << "MediaPipe graph initialized and started";
        return absl::OkStatus();
    }
    
    absl::Status Run() {
        // Open camera
        int camera_id = absl::GetFlag(FLAGS_camera_id);
        cv::VideoCapture capture(camera_id);
        if (!capture.isOpened()) {
            return absl::UnavailableError("Failed to open camera " + std::to_string(camera_id));
        }
        
        // Set camera properties
        capture.set(cv::CAP_PROP_FRAME_WIDTH, 640);
        capture.set(cv::CAP_PROP_FRAME_HEIGHT, 480);
        capture.set(cv::CAP_PROP_FPS, 30);
        
        cv::namedWindow(kWindowName, cv::WINDOW_AUTOSIZE);
        
        ABSL_LOG(INFO) << "Starting capture loop. Press 'q' to quit.";
        
        int max_frames = absl::GetFlag(FLAGS_max_frames);
        bool running = true;
        
        while (running) {
            cv::Mat camera_frame_raw;
            capture >> camera_frame_raw;
            
            if (camera_frame_raw.empty()) {
                ABSL_LOG(WARNING) << "Empty frame from camera";
                continue;
            }
            
            // Convert BGR to RGB and flip for selfie view
            cv::Mat camera_frame;
            cv::cvtColor(camera_frame_raw, camera_frame, cv::COLOR_BGR2RGB);
            cv::flip(camera_frame, camera_frame, 1);  // Horizontal flip
            
            // Create ImageFrame for MediaPipe
            auto input_frame = absl::make_unique<mediapipe::ImageFrame>(
                mediapipe::ImageFormat::SRGB, 
                camera_frame.cols, 
                camera_frame.rows,
                mediapipe::ImageFrame::kDefaultAlignmentBoundary);
            
            // Copy pixel data
            cv::Mat input_frame_mat = mediapipe::formats::MatView(input_frame.get());
            camera_frame.copyTo(input_frame_mat);
            
            // Generate timestamp
            size_t timestamp_us = frame_count_++;
            
            // Send to graph
            auto status = graph_->AddPacketToInputStream(
                kInputStream,
                mediapipe::Adopt(input_frame.release())
                    .At(mediapipe::Timestamp(timestamp_us)));
            
            if (!status.ok()) {
                ABSL_LOG(ERROR) << "Failed to add packet: " << status.message();
                continue;
            }
            
            // Draw visualization on display frame
            cv::Mat display_frame;
            cv::cvtColor(camera_frame, display_frame, cv::COLOR_RGB2BGR);
            DrawLandmarks(display_frame);
            
            // Show stats
            std::string stats = "Frame: " + std::to_string(frame_count_) + 
                              " | Landmarks: " + std::to_string(landmarks_received_) +
                              " | Hands: " + std::to_string(current_hand_count_);
            cv::putText(display_frame, stats, cv::Point(10, 30),
                       cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(0, 255, 0), 2);
            
            cv::imshow(kWindowName, display_frame);
            
            // Check for exit
            int key = cv::waitKey(5);
            if (key == 'q' || key == 27) {  // 'q' or ESC
                running = false;
            }
            
            if (max_frames > 0 && frame_count_ >= max_frames) {
                ABSL_LOG(INFO) << "Reached max frames: " << max_frames;
                running = false;
            }
        }
        
        return absl::OkStatus();
    }
    
    absl::Status Shutdown() {
        if (graph_) {
            ABSL_LOG(INFO) << "Closing input stream...";
            MP_RETURN_IF_ERROR(graph_->CloseInputStream(kInputStream));
            
            ABSL_LOG(INFO) << "Waiting for graph to finish...";
            MP_RETURN_IF_ERROR(graph_->WaitUntilDone());
        }
        
        cv::destroyAllWindows();
        
        ABSL_LOG(INFO) << "Total frames processed: " << frame_count_;
        ABSL_LOG(INFO) << "Total landmark packets received: " << landmarks_received_;
        
        return absl::OkStatus();
    }

private:
    absl::Status ProcessLandmarks(const mediapipe::Packet& packet) {
        auto& multi_hand_landmarks = 
            packet.Get<std::vector<mediapipe::NormalizedLandmarkList>>();
        
        landmarks_received_++;
        
        // Thread-safe update of current landmarks
        std::lock_guard<std::mutex> lock(landmarks_mutex_);
        current_landmarks_ = multi_hand_landmarks;
        current_hand_count_ = multi_hand_landmarks.size();
        
        // Log first detection
        if (landmarks_received_ == 1) {
            ABSL_LOG(INFO) << "First hand landmarks received! Hands detected: " 
                          << current_hand_count_;
            if (!multi_hand_landmarks.empty()) {
                ABSL_LOG(INFO) << "Landmarks per hand: " 
                              << multi_hand_landmarks[0].landmark_size();
            }
        }
        
        return absl::OkStatus();
    }
    
    void DrawLandmarks(cv::Mat& frame) {
        std::lock_guard<std::mutex> lock(landmarks_mutex_);
        
        if (current_landmarks_.empty()) {
            return;
        }
        
        // Hand landmark connections (MediaPipe hand topology)
        static const std::vector<std::pair<int, int>> connections = {
            // Thumb
            {0, 1}, {1, 2}, {2, 3}, {3, 4},
            // Index finger
            {0, 5}, {5, 6}, {6, 7}, {7, 8},
            // Middle finger
            {0, 9}, {9, 10}, {10, 11}, {11, 12},
            // Ring finger
            {0, 13}, {13, 14}, {14, 15}, {15, 16},
            // Pinky
            {0, 17}, {17, 18}, {18, 19}, {19, 20},
            // Palm
            {5, 9}, {9, 13}, {13, 17}
        };
        
        // Draw each detected hand
        for (const auto& hand_landmarks : current_landmarks_) {
            // Convert normalized coordinates to pixel coordinates
            std::vector<cv::Point> points;
            for (const auto& landmark : hand_landmarks.landmark()) {
                int x = static_cast<int>(landmark.x() * frame.cols);
                int y = static_cast<int>(landmark.y() * frame.rows);
                points.push_back(cv::Point(x, y));
            }
            
            // Draw connections
            for (const auto& connection : connections) {
                if (connection.first < points.size() && 
                    connection.second < points.size()) {
                    cv::line(frame, points[connection.first], 
                            points[connection.second], 
                            cv::Scalar(0, 255, 0), 2);
                }
            }
            
            // Draw landmark points
            for (size_t i = 0; i < points.size(); ++i) {
                cv::Scalar color = (i == 0 || i == 4 || i == 8 || i == 12 || i == 16 || i == 20) 
                    ? cv::Scalar(0, 0, 255)    // Red for fingertips and wrist
                    : cv::Scalar(255, 0, 0);   // Blue for other joints
                cv::circle(frame, points[i], 4, color, -1);
            }
        }
    }
    
    std::unique_ptr<mediapipe::CalculatorGraph> graph_;
    size_t frame_count_;
    size_t landmarks_received_;
    
    std::mutex landmarks_mutex_;
    std::vector<mediapipe::NormalizedLandmarkList> current_landmarks_;
    size_t current_hand_count_ = 0;
};

int main(int argc, char** argv) {
    google::InitGoogleLogging(argv[0]);
    absl::ParseCommandLine(argc, argv);
    
    ABSL_LOG(INFO) << "=== MediaPipe Hand Tracking Test ===";
    ABSL_LOG(INFO) << "Graph config: " << absl::GetFlag(FLAGS_graph_config);
    ABSL_LOG(INFO) << "Camera ID: " << absl::GetFlag(FLAGS_camera_id);
    
    HandTrackingTest test;
    
    auto status = test.Initialize();
    if (!status.ok()) {
        ABSL_LOG(ERROR) << "Initialization failed: " << status.message();
        return EXIT_FAILURE;
    }
    
    status = test.Run();
    if (!status.ok()) {
        ABSL_LOG(ERROR) << "Runtime error: " << status.message();
        return EXIT_FAILURE;
    }
    
    status = test.Shutdown();
    if (!status.ok()) {
        ABSL_LOG(ERROR) << "Shutdown error: " << status.message();
        return EXIT_FAILURE;
    }
    
    ABSL_LOG(INFO) << "Test completed successfully!";
    return EXIT_SUCCESS;
}
