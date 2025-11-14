// Hand detection helper node implementation
#include "buttoning_system/hand_detection_node.hpp"

namespace buttoning_system {

HandDetectionNode::HandDetectionNode()
    : Node("hand_detection_node"),
      new_frame_available_(false),
      should_exit_(false),
      hand_detected_(false),
      processing_time_(0.0)
{
    // Create publisher for hand landmarks
    hand_pub_ = this->create_publisher<buttoning_msgs::msg::HandLandmarks>(
        "hand_landmarks", 10);
    
    // Subscribe to camera images (or raw frames)
    image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
        "camera/image_raw", 10,
        std::bind(&HandDetectionNode::imageCallback, this, std::placeholders::_1));
    
    // TODO: Initialize MediaPipe hand detector
    // Set up MediaPipe graph for hand detection
    // hand_graph_ = std::make_unique<mediapipe::CalculatorGraph>();
    // Load hand detection model configuration
    
    // PLACEHOLDER: In real implementation, initialize MediaPipe:
    // 1. Load hand_detection.pbtxt graph configuration
    // 2. Set up input/output streams
    // 3. Start the graph
    
    // Start background detection thread
    detection_thread_ = std::thread(&HandDetectionNode::detectionThreadWorker, this);
    
    RCLCPP_INFO(this->get_logger(), "Hand detection node initialized");
}

HandDetectionNode::~HandDetectionNode() {
    // Signal thread to exit
    {
        std::lock_guard<std::mutex> lock(frame_mutex_);
        should_exit_ = true;
        new_frame_available_ = true;
    }
    frame_cv_.notify_one();
    
    // Wait for thread to finish
    if (detection_thread_.joinable()) {
        detection_thread_.join();
    }
    
    // TODO: Cleanup MediaPipe resources
    // hand_graph_->CloseAllPacketSources();
    // hand_graph_->WaitUntilDone();
    
    RCLCPP_INFO(this->get_logger(), "Hand detection node shutting down");
}

void HandDetectionNode::imageCallback(const sensor_msgs::msg::Image::SharedPtr msg) {
    try {
        // Convert ROS image message to OpenCV Mat
        cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::RGB8);
        
        // Lock and update the frame buffer
        {
            std::lock_guard<std::mutex> lock(frame_mutex_);
            current_frame_ = cv_ptr->image.clone();
            new_frame_available_ = true;
        }
        
        // Notify detection thread
        frame_cv_.notify_one();
        
    } catch (cv_bridge::Exception& e) {
        RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
    }
}

void HandDetectionNode::detectionThreadWorker() {
    RCLCPP_INFO(this->get_logger(), "Hand detection thread started");
    
    while (!should_exit_) {
        // Wait for new frame
        std::unique_lock<std::mutex> lock(frame_mutex_);
        frame_cv_.wait(lock, [this] { 
            return new_frame_available_ || should_exit_; 
        });
        
        if (should_exit_) {
            break;
        }
        
        if (new_frame_available_ && !current_frame_.empty()) {
            // Copy frame for processing
            processing_frame_ = current_frame_.clone();
            new_frame_available_ = false;
            lock.unlock();
            
            // Process hands (outside of lock)
            processHands(processing_frame_);
        } else {
            lock.unlock();
        }
    }
    
    RCLCPP_INFO(this->get_logger(), "Hand detection thread exiting");
}

void HandDetectionNode::processHands(const cv::Mat& frame) {
    auto start_time = std::chrono::high_resolution_clock::now();
    
    // TODO: Run MediaPipe hand detection
    // 1. Convert cv::Mat to mediapipe::ImageFrame
    // 2. Send frame to MediaPipe graph
    // 3. Retrieve hand landmarks from output stream
    
    // PLACEHOLDER: Simulate hand detection
    // In real implementation:
    // - mediapipe::ImageFrame mp_frame(mediapipe::ImageFormat::SRGB, 
    //                                   frame.cols, frame.rows, ...);
    // - hand_graph_->AddPacketToInputStream(...);
    // - auto output_packets = hand_graph_->GetOutputSidePackets();
    // - Parse landmarks from output packets
    
    hand_detected_ = false; // Placeholder
    hand_landmarks_.clear();
    
    // Simulate detecting hand with 21 landmarks
    // if (/* hand detected in frame */) {
    //     hand_detected_ = true;
    //     for (int i = 0; i < 21; ++i) {
    //         hand_landmarks_.push_back({0.0, 0.0, 0.0}); // x, y, z
    //     }
    // }
    
    auto end_time = std::chrono::high_resolution_clock::now();
    processing_time_ = std::chrono::duration<double>(end_time - start_time).count();
    
    // Publish hand landmarks
    auto hand_msg = std::make_unique<buttoning_msgs::msg::HandLandmarks>();
    hand_msg->header.stamp = this->now();
    hand_msg->header.frame_id = "camera_frame";
    hand_msg->hand_detected = hand_detected_;
    hand_msg->processing_time = processing_time_;
    
    if (hand_detected_) {
        for (const auto& landmark : hand_landmarks_) {
            geometry_msgs::msg::Point pt;
            pt.x = landmark.x;
            pt.y = landmark.y;
            pt.z = landmark.z;
            hand_msg->landmarks.push_back(pt);
        }
    }
    
    hand_pub_->publish(std::move(hand_msg));
    
    RCLCPP_DEBUG(this->get_logger(), 
                 "Hand detection completed in %.3f seconds, detected: %d",
                 processing_time_, hand_detected_);
}

} // namespace buttoning_system

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<buttoning_system::HandDetectionNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
