// Main detection node for button/buttonhole detection using camera and AI model
// Corresponds to the main loop in the Python notebook

#ifndef DETECTION_NODE_HPP_
#define DETECTION_NODE_HPP_

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <buttoning_msgs/msg/detection.hpp>
#include <buttoning_msgs/msg/hand_landmarks.hpp>
#if defined(ROS_DISTRO_HUMBLE)
#include <cv_bridge/cv_bridge.h>
#else
#include <cv_bridge/cv_bridge.hpp>
#endif
#include <opencv2/opencv.hpp>
#include <memory>
#include <vector>

#include "buttoning_system/sort_tracker.hpp"

// Placeholder for detectron2-like inference (use ONNX Runtime or TensorRT)
// #include <onnxruntime_cxx_api.h>
// #include <tensorrt/NvInfer.h>

namespace buttoning_system {

class DetectionNode : public rclcpp::Node {
public:
    DetectionNode();
    ~DetectionNode();

private:
    void mainLoop();
    void processFrame();
    void runObjectDetection(const cv::Mat& frame);
    void applyPerClassThreshold();
    void runTracking();
    void handLandmarksCallback(const buttoning_msgs::msg::HandLandmarks::SharedPtr msg);
    void imageCallback(const sensor_msgs::msg::Image::SharedPtr msg);
    
    // Publishers
    rclcpp::Publisher<buttoning_msgs::msg::Detection>::SharedPtr detection_pub_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr debug_image_pub_;
    
    // Subscribers
    rclcpp::Subscription<buttoning_msgs::msg::HandLandmarks>::SharedPtr hand_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
    
    // Timer for main loop
    rclcpp::TimerBase::SharedPtr main_timer_;
    
    // Detection model (placeholder for ONNX/TensorRT)
    // std::unique_ptr<Ort::Session> onnx_session_;
    // std::unique_ptr<nvinfer1::ICudaEngine> trt_engine_;
    
    // SORT tracker
    std::unique_ptr<SortTracker> tracker_;
    
    // State variables
    cv::Mat current_frame_;
    cv::Mat depth_frame_;
    std::vector<cv::Rect> pred_boxes_;
    std::vector<int> pred_classes_;
    std::vector<float> pred_scores_;
    std::vector<cv::Point2i> centers_;
    std::vector<int> track_ids_;
    
    bool hand_detected_;
    bool frame_received_;
    buttoning_msgs::msg::HandLandmarks::SharedPtr latest_hand_landmarks_;
    
    // Parameters
    int max_detections_;
    double button_threshold_;
    double buttonhole_threshold_;
    double iou_threshold_;
    int max_age_;
    int min_hits_;
    
    // Camera intrinsics
    cv::Mat camera_matrix_;
    cv::Mat dist_coeffs_;
    
    // Performance metrics
    double preproc_time_;
    double inference_time_;
    double tracking_time_;
    
    int not_found_counter_;
};

} // namespace buttoning_system

#endif // DETECTION_NODE_HPP_
