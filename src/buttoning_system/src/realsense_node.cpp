// RealSense camera node implementation
#include "buttoning_system/realsense_node.hpp"
#include <chrono>

namespace buttoning_system {

RealSenseNode::RealSenseNode()
    : Node("realsense_node"),
      capture_time_(0.0),
      frame_count_(0)
{
    // Declare and get parameters
    this->declare_parameter<int>("color_width", 640);
    this->declare_parameter<int>("color_height", 480);
    this->declare_parameter<int>("depth_width", 640);
    this->declare_parameter<int>("depth_height", 480);
    this->declare_parameter<int>("fps", 30);
    this->declare_parameter<bool>("enable_depth", true);
    this->declare_parameter<bool>("enable_color", true);
    this->declare_parameter<bool>("align_depth_to_color", true);
    this->declare_parameter<std::string>("camera_name", "camera");
    this->declare_parameter<std::string>("serial_number", "");
    
    this->get_parameter("color_width", color_width_);
    this->get_parameter("color_height", color_height_);
    this->get_parameter("depth_width", depth_width_);
    this->get_parameter("depth_height", depth_height_);
    this->get_parameter("fps", fps_);
    this->get_parameter("enable_depth", enable_depth_);
    this->get_parameter("enable_color", enable_color_);
    this->get_parameter("align_depth_to_color", align_depth_to_color_);
    this->get_parameter("camera_name", camera_name_);
    this->get_parameter("serial_number", serial_number_);
    
    // Create publishers
    if (enable_color_) {
        color_image_pub_ = this->create_publisher<sensor_msgs::msg::Image>(
            camera_name_ + "/color/image_raw", 10);
        color_info_pub_ = this->create_publisher<sensor_msgs::msg::CameraInfo>(
            camera_name_ + "/color/camera_info", 10);
    }
    
    if (enable_depth_) {
        depth_image_pub_ = this->create_publisher<sensor_msgs::msg::Image>(
            camera_name_ + "/depth/image_raw", 10);
        depth_info_pub_ = this->create_publisher<sensor_msgs::msg::CameraInfo>(
            camera_name_ + "/depth/camera_info", 10);
            
        if (align_depth_to_color_ && enable_color_) {
            aligned_depth_pub_ = this->create_publisher<sensor_msgs::msg::Image>(
                camera_name_ + "/aligned_depth_to_color/image_raw", 10);
        }
    }
    
    // Configure RealSense pipeline
    try {
        if (enable_color_) {
            config_.enable_stream(RS2_STREAM_COLOR, color_width_, color_height_, 
                                RS2_FORMAT_RGB8, fps_);
        }
        
        if (enable_depth_) {
            config_.enable_stream(RS2_STREAM_DEPTH, depth_width_, depth_height_, 
                                RS2_FORMAT_Z16, fps_);
        }
        
        // Enable specific device by serial number if provided
        if (!serial_number_.empty()) {
            config_.enable_device(serial_number_);
            RCLCPP_INFO(this->get_logger(), "Enabling RealSense device with serial: %s", 
                       serial_number_.c_str());
        }
        
        // Start the pipeline
        profile_ = pipeline_.start(config_);
        
        // Create alignment object if needed
        if (enable_depth_ && enable_color_ && align_depth_to_color_) {
            align_to_color_ = std::make_unique<rs2::align>(RS2_STREAM_COLOR);
        }
        
        // Setup camera info messages
        setupCameraInfo();
        
        // Create timer for frame publishing (based on FPS)
        auto timer_period = std::chrono::milliseconds(1000 / fps_);
        frame_timer_ = this->create_wall_timer(
            timer_period,
            std::bind(&RealSenseNode::publishFrames, this));
        
        RCLCPP_INFO(this->get_logger(), "RealSense node initialized successfully");
        RCLCPP_INFO(this->get_logger(), "  Color: %dx%d @ %d fps (enabled: %s)", 
                   color_width_, color_height_, fps_, enable_color_ ? "yes" : "no");
        RCLCPP_INFO(this->get_logger(), "  Depth: %dx%d @ %d fps (enabled: %s)", 
                   depth_width_, depth_height_, fps_, enable_depth_ ? "yes" : "no");
        RCLCPP_INFO(this->get_logger(), "  Align depth to color: %s", 
                   align_depth_to_color_ ? "yes" : "no");
        
    } catch (const rs2::error& e) {
        RCLCPP_ERROR(this->get_logger(), "RealSense error calling %s(%s): %s",
                    e.get_failed_function().c_str(),
                    e.get_failed_args().c_str(),
                    e.what());
        throw;
    } catch (const std::exception& e) {
        RCLCPP_ERROR(this->get_logger(), "Exception initializing RealSense: %s", e.what());
        throw;
    }
}

RealSenseNode::~RealSenseNode() {
    try {
        pipeline_.stop();
        RCLCPP_INFO(this->get_logger(), "RealSense pipeline stopped");
    } catch (const rs2::error& e) {
        RCLCPP_ERROR(this->get_logger(), "Error stopping pipeline: %s", e.what());
    }
}

void RealSenseNode::setupCameraInfo() {
    try {
        // Get color stream profile
        if (enable_color_) {
            auto color_stream = profile_.get_stream(RS2_STREAM_COLOR)
                                       .as<rs2::video_stream_profile>();
            color_camera_info_ = createCameraInfo(color_stream);
        }
        
        // Get depth stream profile
        if (enable_depth_) {
            auto depth_stream = profile_.get_stream(RS2_STREAM_DEPTH)
                                       .as<rs2::video_stream_profile>();
            depth_camera_info_ = createCameraInfo(depth_stream);
        }
    } catch (const rs2::error& e) {
        RCLCPP_ERROR(this->get_logger(), "Error getting camera info: %s", e.what());
    }
}

sensor_msgs::msg::CameraInfo RealSenseNode::createCameraInfo(
    const rs2::video_stream_profile& profile) {
    
    sensor_msgs::msg::CameraInfo info;
    auto intrinsics = profile.get_intrinsics();
    
    info.width = intrinsics.width;
    info.height = intrinsics.height;
    
    // Distortion model
    info.distortion_model = "plumb_bob";
    
    // Distortion coefficients [k1, k2, t1, t2, k3]
    info.d.resize(5);
    for (int i = 0; i < 5; i++) {
        info.d[i] = (i < 5) ? intrinsics.coeffs[i] : 0.0;
    }
    
    // Intrinsic camera matrix [fx 0 cx; 0 fy cy; 0 0 1]
    info.k[0] = intrinsics.fx;
    info.k[1] = 0.0;
    info.k[2] = intrinsics.ppx;
    info.k[3] = 0.0;
    info.k[4] = intrinsics.fy;
    info.k[5] = intrinsics.ppy;
    info.k[6] = 0.0;
    info.k[7] = 0.0;
    info.k[8] = 1.0;
    
    // Rectification matrix (identity for monocular camera)
    info.r[0] = 1.0; info.r[1] = 0.0; info.r[2] = 0.0;
    info.r[3] = 0.0; info.r[4] = 1.0; info.r[5] = 0.0;
    info.r[6] = 0.0; info.r[7] = 0.0; info.r[8] = 1.0;
    
    // Projection matrix [fx' 0 cx' Tx; 0 fy' cy' Ty; 0 0 1 0]
    info.p[0] = intrinsics.fx;
    info.p[1] = 0.0;
    info.p[2] = intrinsics.ppx;
    info.p[3] = 0.0;
    info.p[4] = 0.0;
    info.p[5] = intrinsics.fy;
    info.p[6] = intrinsics.ppy;
    info.p[7] = 0.0;
    info.p[8] = 0.0;
    info.p[9] = 0.0;
    info.p[10] = 1.0;
    info.p[11] = 0.0;
    
    return info;
}

void RealSenseNode::publishFrames() {
    try {
        auto start_time = std::chrono::high_resolution_clock::now();
        
        // Wait for frames with timeout
        rs2::frameset frames = pipeline_.wait_for_frames(5000);
        
        // Get timestamp
        auto timestamp = this->get_clock()->now();
        
        // Align frames if requested
        if (align_to_color_ && enable_depth_ && enable_color_) {
            frames = align_to_color_->process(frames);
        }
        
        // Publish color frame
        if (enable_color_ && frames.get_color_frame()) {
            rs2::video_frame color_frame = frames.get_color_frame();
            
            // Create OpenCV Mat from RealSense frame
            cv::Mat color_mat(cv::Size(color_width_, color_height_), 
                            CV_8UC3, 
                            (void*)color_frame.get_data(), 
                            cv::Mat::AUTO_STEP);
            
            // Convert to ROS message
            std_msgs::msg::Header header;
            header.stamp = timestamp;
            header.frame_id = camera_name_ + "_color_optical_frame";
            
            sensor_msgs::msg::Image::SharedPtr color_msg = 
                cv_bridge::CvImage(header, sensor_msgs::image_encodings::RGB8, color_mat).toImageMsg();
            
            // Publish image and camera info
            color_image_pub_->publish(*color_msg);
            
            color_camera_info_.header = header;
            color_info_pub_->publish(color_camera_info_);
        }
        
        // Publish depth frame
        if (enable_depth_ && frames.get_depth_frame()) {
            rs2::depth_frame depth_frame = frames.get_depth_frame();
            
            // Create OpenCV Mat from RealSense depth frame
            cv::Mat depth_mat(cv::Size(depth_width_, depth_height_), 
                            CV_16UC1, 
                            (void*)depth_frame.get_data(), 
                            cv::Mat::AUTO_STEP);
            
            // Convert to ROS message
            std_msgs::msg::Header header;
            header.stamp = timestamp;
            header.frame_id = camera_name_ + "_depth_optical_frame";
            
            sensor_msgs::msg::Image::SharedPtr depth_msg = 
                cv_bridge::CvImage(header, sensor_msgs::image_encodings::TYPE_16UC1, depth_mat).toImageMsg();
            
            // Publish image and camera info
            depth_image_pub_->publish(*depth_msg);
            
            depth_camera_info_.header = header;
            depth_info_pub_->publish(depth_camera_info_);
            
            // Publish aligned depth if available
            if (aligned_depth_pub_ && align_depth_to_color_) {
                header.frame_id = camera_name_ + "_color_optical_frame";
                sensor_msgs::msg::Image::SharedPtr aligned_depth_msg = 
                    cv_bridge::CvImage(header, sensor_msgs::image_encodings::TYPE_16UC1, depth_mat).toImageMsg();
                aligned_depth_pub_->publish(*aligned_depth_msg);
            }
        }
        
        auto end_time = std::chrono::high_resolution_clock::now();
        capture_time_ = std::chrono::duration<double, std::milli>(end_time - start_time).count();
        
        frame_count_++;
        if (frame_count_ % 100 == 0) {
            RCLCPP_DEBUG(this->get_logger(), "Frame %d published (capture time: %.2f ms)", 
                        frame_count_, capture_time_);
        }
        
    } catch (const rs2::error& e) {
        RCLCPP_ERROR(this->get_logger(), "RealSense error: %s", e.what());
    } catch (const std::exception& e) {
        RCLCPP_ERROR(this->get_logger(), "Exception in publishFrames: %s", e.what());
    }
}

} // namespace buttoning_system

// Main function
int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<buttoning_system::RealSenseNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
