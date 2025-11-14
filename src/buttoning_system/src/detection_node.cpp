// Main detection node implementation
#include "buttoning_system/detection_node.hpp"

namespace buttoning_system {

DetectionNode::DetectionNode() 
    : Node("detection_node"),
      max_detections_(10),
      button_threshold_(0.8),
      buttonhole_threshold_(0.7),
      iou_threshold_(0.01),
      max_age_(10),
      min_hits_(1),
      not_found_counter_(0),
      hand_detected_(false),
      frame_received_(false)
{
    // Declare and get parameters
    this->declare_parameter("max_detections", 10);
    this->declare_parameter("button_threshold", 0.8);
    this->declare_parameter("buttonhole_threshold", 0.7);
    this->declare_parameter("iou_threshold", 0.01);
    this->declare_parameter("model_path", "");
    
    max_detections_ = this->get_parameter("max_detections").as_int();
    button_threshold_ = this->get_parameter("button_threshold").as_double();
    buttonhole_threshold_ = this->get_parameter("buttonhole_threshold").as_double();
    iou_threshold_ = this->get_parameter("iou_threshold").as_double();
    
    // Create publishers
    detection_pub_ = this->create_publisher<buttoning_msgs::msg::Detection>(
        "detections", 10);
    debug_image_pub_ = this->create_publisher<sensor_msgs::msg::Image>(
        "debug_image", 10);
    
    // Create subscriber for camera image (topic name can be remapped in launch file)
    image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
        "image_raw", 10,
        std::bind(&DetectionNode::imageCallback, this, std::placeholders::_1));
    
    hand_sub_ = this->create_subscription<buttoning_msgs::msg::HandLandmarks>(
        "hand_landmarks", 10,
        std::bind(&DetectionNode::handLandmarksCallback, this, std::placeholders::_1));
    
    // TODO: Initialize detection model (ONNX Runtime or TensorRT)
    // Load model from parameter "model_path"
    // onnx_session_ = std::make_unique<Ort::Session>(...);
    
    // Initialize SORT tracker
    tracker_ = std::make_unique<SortTracker>(max_age_, min_hits_, iou_threshold_);
    
    // Create timer for main loop (30 Hz)
    main_timer_ = this->create_wall_timer(
        std::chrono::milliseconds(33),
        std::bind(&DetectionNode::mainLoop, this));
    
    RCLCPP_INFO(this->get_logger(), "Detection node initialized");
}

DetectionNode::~DetectionNode() {
    RCLCPP_INFO(this->get_logger(), "Detection node shutting down");
}

void DetectionNode::imageCallback(const sensor_msgs::msg::Image::SharedPtr msg) {
    try {
        cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::RGB8);
        current_frame_ = cv_ptr->image.clone();
        frame_received_ = true;
    } catch (cv_bridge::Exception& e) {
        RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
    }
}

void DetectionNode::mainLoop() {
    if (!frame_received_) {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                             "Waiting for images on 'image_raw' topic...");
        return;
    }
    
    processFrame();
}

void DetectionNode::processFrame() {
    auto start_preproc = std::chrono::high_resolution_clock::now();
    
    // Preprocessing - crop to valid data region
    cv::Mat valid_mask;
    cv::reduce(current_frame_, valid_mask, 1, cv::REDUCE_SUM);
    
    // TODO: Find bounding box of valid region
    // Crop frame to valid region
    cv::Mat cropped_frame = current_frame_; // Placeholder
    
    auto end_preproc = std::chrono::high_resolution_clock::now();
    preproc_time_ = std::chrono::duration<double>(end_preproc - start_preproc).count();
    
    // Run object detection
    runObjectDetection(cropped_frame);
}

void DetectionNode::runObjectDetection(const cv::Mat& frame) {
    auto start_inference = std::chrono::high_resolution_clock::now();
    
    // TODO: Run inference using ONNX Runtime or TensorRT
    // 1. Preprocess image: RGB->BGR, normalize, convert to tensor
    // 2. Run model inference
    // 3. Post-process outputs to get boxes, classes, scores
    
    // PLACEHOLDER: Clear previous detections
    pred_boxes_.clear();
    pred_classes_.clear();
    pred_scores_.clear();
    centers_.clear();
    
    // PLACEHOLDER: Simulate some detections
    // In real implementation:
    // - Convert frame to model input format
    // - Run onnx_session_->Run(...) or trt_engine_->execute(...)
    // - Parse output tensors to get bounding boxes, classes, scores
    
    auto end_inference = std::chrono::high_resolution_clock::now();
    inference_time_ = std::chrono::duration<double>(end_inference - start_inference).count();
    
    if (pred_boxes_.empty()) {
        not_found_counter_++;
        RCLCPP_DEBUG(this->get_logger(), "No objects detected");
        return;
    }
    
    // Apply per-class thresholding
    applyPerClassThreshold();
    
    // Calculate centers
    for (const auto& box : pred_boxes_) {
        int cx = (box.x + box.x + box.width) / 2;
        int cy = (box.y + box.y + box.height) / 2;
        centers_.push_back(cv::Point2i(cx, cy));
    }
    
    // Run tracking
    runTracking();
    
    // Publish detections
    auto detection_msg = std::make_unique<buttoning_msgs::msg::Detection>();
    detection_msg->header.stamp = this->now();
    detection_msg->header.frame_id = "camera_frame";
    
    for (size_t i = 0; i < pred_boxes_.size(); ++i) {
        detection_msg->boxes.push_back(pred_boxes_[i].x);
        detection_msg->boxes.push_back(pred_boxes_[i].y);
        detection_msg->boxes.push_back(pred_boxes_[i].x + pred_boxes_[i].width);
        detection_msg->boxes.push_back(pred_boxes_[i].y + pred_boxes_[i].height);
        detection_msg->classes.push_back(pred_classes_[i]);
        detection_msg->scores.push_back(pred_scores_[i]);
        detection_msg->centers.push_back(centers_[i].x);
        detection_msg->centers.push_back(centers_[i].y);
        if (i < track_ids_.size()) {
            detection_msg->track_ids.push_back(track_ids_[i]);
        }
    }
    
    detection_pub_->publish(std::move(detection_msg));
    
    RCLCPP_DEBUG(this->get_logger(), 
                 "Preproc: %.3f, Inference: %.3f, Tracking: %.3f",
                 preproc_time_, inference_time_, tracking_time_);
}

void DetectionNode::applyPerClassThreshold() {
    std::vector<cv::Rect> filtered_boxes;
    std::vector<int> filtered_classes;
    std::vector<float> filtered_scores;
    
    for (size_t i = 0; i < pred_boxes_.size(); ++i) {
        float threshold = (pred_classes_[i] == 0) ? button_threshold_ : buttonhole_threshold_;
        if (pred_scores_[i] >= threshold) {
            filtered_boxes.push_back(pred_boxes_[i]);
            filtered_classes.push_back(pred_classes_[i]);
            filtered_scores.push_back(pred_scores_[i]);
        }
    }
    
    pred_boxes_ = filtered_boxes;
    pred_classes_ = filtered_classes;
    pred_scores_ = filtered_scores;
}

void DetectionNode::runTracking() {
    auto start_tracking = std::chrono::high_resolution_clock::now();
    
    // Run SORT tracking
    auto tracked_objects = tracker_->update(pred_boxes_, pred_scores_);
    
    // Get track IDs in order of detections
    track_ids_ = tracker_->getTrackIds();
    
    auto end_tracking = std::chrono::high_resolution_clock::now();
    tracking_time_ = std::chrono::duration<double>(end_tracking - start_tracking).count();
    
    RCLCPP_DEBUG(this->get_logger(), 
                 "Tracking: %zu detections, %zu active tracks",
                 pred_boxes_.size(), tracker_->getNumTracks());
}

void DetectionNode::handLandmarksCallback(
    const buttoning_msgs::msg::HandLandmarks::SharedPtr msg) 
{
    latest_hand_landmarks_ = msg;
    hand_detected_ = msg->hand_detected;
    
    RCLCPP_DEBUG(this->get_logger(), "Received hand landmarks, detected: %d", 
                 hand_detected_);
}

} // namespace buttoning_system

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<buttoning_system::DetectionNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
