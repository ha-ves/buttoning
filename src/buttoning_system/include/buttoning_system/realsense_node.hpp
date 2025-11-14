// RealSense camera node for publishing RGB and depth images
// Provides camera streams for the buttoning system

#ifndef REALSENSE_NODE_HPP_
#define REALSENSE_NODE_HPP_

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <cv_bridge/cv_bridge.hpp>
#include <opencv2/opencv.hpp>
#include <memory>
#include <librealsense2/rs.hpp>

namespace buttoning_system {

class RealSenseNode : public rclcpp::Node {
public:
    RealSenseNode();
    ~RealSenseNode();

private:
    void publishFrames();
    void setupCameraInfo();
    sensor_msgs::msg::CameraInfo createCameraInfo(const rs2::video_stream_profile& profile);
    
    // Publishers
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr color_image_pub_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr depth_image_pub_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr aligned_depth_pub_;
    rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr color_info_pub_;
    rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr depth_info_pub_;
    
    // Timer for publishing frames
    rclcpp::TimerBase::SharedPtr frame_timer_;
    
    // RealSense pipeline and configuration
    rs2::pipeline pipeline_;
    rs2::config config_;
    rs2::pipeline_profile profile_;
    
    // Align depth to color frames
    std::unique_ptr<rs2::align> align_to_color_;
    
    // Camera intrinsics
    sensor_msgs::msg::CameraInfo color_camera_info_;
    sensor_msgs::msg::CameraInfo depth_camera_info_;
    
    // Parameters
    int color_width_;
    int color_height_;
    int depth_width_;
    int depth_height_;
    int fps_;
    bool enable_depth_;
    bool enable_color_;
    bool align_depth_to_color_;
    std::string camera_name_;
    std::string serial_number_;
    
    // Performance tracking
    double capture_time_;
    int frame_count_;
};

} // namespace buttoning_system

#endif // REALSENSE_NODE_HPP_
