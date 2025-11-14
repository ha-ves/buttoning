// Robot arm controller node for Kinova arms
// Handles dual arm coordination for buttoning tasks

#ifndef ARM_CONTROLLER_NODE_HPP_
#define ARM_CONTROLLER_NODE_HPP_

#include <rclcpp/rclcpp.hpp>
#include <buttoning_msgs/msg/robot_command.hpp>
#include <buttoning_msgs/msg/detection.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <std_msgs/msg/bool.hpp>
#include <memory>
#include <mutex>
#include <thread>

// Placeholder for Kinova API (use actual kortex_api headers)
// #include <kortex_api/BaseClient.h>
// #include <kortex_api/BaseCyclicClient.h>
// #include <kortex_api/SessionManager.h>

namespace buttoning_system {

class ArmControllerNode : public rclcpp::Node {
public:
    ArmControllerNode();
    ~ArmControllerNode();

private:
    void detectionCallback(const buttoning_msgs::msg::Detection::SharedPtr msg);
    void commandCallback(const buttoning_msgs::msg::RobotCommand::SharedPtr msg);
    void executeMotion();
    void moveToPosition(const geometry_msgs::msg::Point& position,
                       const geometry_msgs::msg::Quaternion& orientation);
    void getJointAngles(std::vector<double>& angles);
    
    // Publishers
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr left_arm_pose_pub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr right_arm_pose_pub_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr motion_complete_pub_;
    
    // Subscribers
    rclcpp::Subscription<buttoning_msgs::msg::Detection>::SharedPtr detection_sub_;
    rclcpp::Subscription<buttoning_msgs::msg::RobotCommand>::SharedPtr command_sub_;
    
    // Kinova arm clients (placeholder)
    // std::unique_ptr<Kinova::Api::BaseClient> left_arm_base_;
    // std::unique_ptr<Kinova::Api::BaseClient> right_arm_base_;
    // std::unique_ptr<Kinova::Api::BaseCyclicClient> left_arm_cyclic_;
    // std::unique_ptr<Kinova::Api::BaseCyclicClient> right_arm_cyclic_;
    
    // State
    std::mutex state_mutex_;
    buttoning_msgs::msg::Detection::SharedPtr latest_detection_;
    buttoning_msgs::msg::RobotCommand::SharedPtr pending_command_;
    
    bool left_arm_connected_;
    bool right_arm_connected_;
    bool motion_in_progress_;
    
    // Parameters
    std::string left_arm_ip_;
    std::string right_arm_ip_;
    std::string arm_username_;
    std::string arm_password_;
    
    // Calibration data
    std::vector<std::vector<double>> tf_frame6_camera_;
};

} // namespace buttoning_system

#endif // ARM_CONTROLLER_NODE_HPP_
