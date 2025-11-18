// Hand detection helper node using MediaPipe
// Corresponds to the detect_hands thread helper in Python notebook

#ifndef HAND_DETECTION_NODE_HPP_
#define HAND_DETECTION_NODE_HPP_

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <buttoning_msgs/msg/hand_landmarks.hpp>
#if defined(ROS_DISTRO_HUMBLE)
#include <cv_bridge/cv_bridge.h>
#else
#include <cv_bridge/cv_bridge.hpp>
#endif
#include <opencv2/opencv.hpp>
#include <memory>
#include <thread>
#include <mutex>
#include <condition_variable>

// Placeholder for MediaPipe (use actual MediaPipe C++ API or Python bindings via pybind11)
// #include <mediapipe/framework/calculator_framework.h>
// #include <mediapipe/framework/formats/image_frame.h>

namespace buttoning_system {

class HandDetectionNode : public rclcpp::Node {
public:
    HandDetectionNode();
    ~HandDetectionNode();

private:
    void imageCallback(const sensor_msgs::msg::Image::SharedPtr msg);
    void detectionThreadWorker();
    void processHands(const cv::Mat& frame);
    
    // Publishers
    rclcpp::Publisher<buttoning_msgs::msg::HandLandmarks>::SharedPtr hand_pub_;
    
    // Subscribers
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
    
    // Threading components
    std::thread detection_thread_;
    std::mutex frame_mutex_;
    std::condition_variable frame_cv_;
    bool new_frame_available_;
    bool should_exit_;
    
    // Current frame buffer
    cv::Mat current_frame_;
    cv::Mat processing_frame_;
    
    // MediaPipe hand detector (placeholder)
    // std::unique_ptr<mediapipe::CalculatorGraph> hand_graph_;
    
    // Hand detection results
    struct HandLandmark {
        double x, y, z;
    };
    std::vector<HandLandmark> hand_landmarks_;
    bool hand_detected_;
    
    // Performance tracking
    double processing_time_;
};

} // namespace buttoning_system

#endif // HAND_DETECTION_NODE_HPP_
