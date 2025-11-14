// Robot arm controller node implementation
#include "buttoning_system/arm_controller_node.hpp"

namespace buttoning_system {

ArmControllerNode::ArmControllerNode()
    : Node("arm_controller_node"),
      left_arm_connected_(false),
      right_arm_connected_(false),
      motion_in_progress_(false)
{
    // Declare and get parameters
    this->declare_parameter("left_arm_ip", "192.168.2.10");
    this->declare_parameter("right_arm_ip", "192.168.2.12");
    this->declare_parameter("arm_username", "admin");
    this->declare_parameter("arm_password", "admin");
    
    left_arm_ip_ = this->get_parameter("left_arm_ip").as_string();
    right_arm_ip_ = this->get_parameter("right_arm_ip").as_string();
    arm_username_ = this->get_parameter("arm_username").as_string();
    arm_password_ = this->get_parameter("arm_password").as_string();
    
    // Create publishers
    left_arm_pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(
        "left_arm/pose", 10);
    right_arm_pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(
        "right_arm/pose", 10);
    motion_complete_pub_ = this->create_publisher<std_msgs::msg::Bool>(
        "motion_complete", 10);
    
    // Create subscribers
    detection_sub_ = this->create_subscription<buttoning_msgs::msg::Detection>(
        "detections", 10,
        std::bind(&ArmControllerNode::detectionCallback, this, std::placeholders::_1));
    command_sub_ = this->create_subscription<buttoning_msgs::msg::RobotCommand>(
        "robot_command", 10,
        std::bind(&ArmControllerNode::commandCallback, this, std::placeholders::_1));
    
    // TODO: Initialize Kinova arm connections
    // Connect to left arm
    // auto left_transport = std::make_shared<Kinova::Api::TCPTransport>();
    // left_transport->connect(left_arm_ip_, 10000);
    // auto left_router = std::make_shared<Kinova::Api::RouterClient>(left_transport);
    // left_router->SetSessionManager(...);
    // left_arm_base_ = std::make_unique<Kinova::Api::BaseClient>(left_router);
    // left_arm_cyclic_ = std::make_unique<Kinova::Api::BaseCyclicClient>(left_router);
    
    // Connect to right arm (similar to left)
    // right_arm_base_ = ...
    // right_arm_cyclic_ = ...
    
    // Set servoing mode
    // Kinova::Api::ServoingModeInformation servo_mode;
    // servo_mode.set_servoing_mode(Kinova::Api::SINGLE_LEVEL_SERVOING);
    // left_arm_base_->SetServoingMode(servo_mode);
    // right_arm_base_->SetServoingMode(servo_mode);
    
    // PLACEHOLDER: Mark arms as connected (in real implementation, check connection status)
    left_arm_connected_ = true;
    right_arm_connected_ = true;
    
    RCLCPP_INFO(this->get_logger(), "Arm controller node initialized");
    RCLCPP_INFO(this->get_logger(), "Left arm: %s, Right arm: %s", 
                left_arm_ip_.c_str(), right_arm_ip_.c_str());
}

ArmControllerNode::~ArmControllerNode() {
    // TODO: Cleanup arm connections
    // left_arm_base_.reset();
    // right_arm_base_.reset();
    // left_arm_cyclic_.reset();
    // right_arm_cyclic_.reset();
    
    RCLCPP_INFO(this->get_logger(), "Arm controller node shutting down");
}

void ArmControllerNode::detectionCallback(
    const buttoning_msgs::msg::Detection::SharedPtr msg)
{
    std::lock_guard<std::mutex> lock(state_mutex_);
    latest_detection_ = msg;
    
    RCLCPP_DEBUG(this->get_logger(), "Received detection with %zu objects",
                 msg->boxes.size() / 4);
    
    // TODO: Process detection and plan motion
    // - Identify button and buttonhole pairs
    // - Calculate grasp poses
    // - Plan dual-arm coordination
}

void ArmControllerNode::commandCallback(
    const buttoning_msgs::msg::RobotCommand::SharedPtr msg)
{
    std::lock_guard<std::mutex> lock(state_mutex_);
    pending_command_ = msg;
    
    RCLCPP_INFO(this->get_logger(), "Received robot command type: %d", 
                msg->command_type);
    
    // Execute motion asynchronously
    std::thread(&ArmControllerNode::executeMotion, this).detach();
}

void ArmControllerNode::executeMotion() {
    if (motion_in_progress_) {
        RCLCPP_WARN(this->get_logger(), "Motion already in progress, ignoring command");
        return;
    }
    
    motion_in_progress_ = true;
    
    buttoning_msgs::msg::RobotCommand::SharedPtr cmd;
    {
        std::lock_guard<std::mutex> lock(state_mutex_);
        cmd = pending_command_;
    }
    
    if (!cmd) {
        motion_in_progress_ = false;
        return;
    }
    
    switch (cmd->command_type) {
        case 0: // Move
            moveToPosition(cmd->position, cmd->orientation);
            break;
        case 1: // Grasp
            RCLCPP_INFO(this->get_logger(), "Executing grasp");
            // TODO: Implement grasp logic
            break;
        case 2: // Release
            RCLCPP_INFO(this->get_logger(), "Executing release");
            // TODO: Implement release logic
            break;
        default:
            RCLCPP_WARN(this->get_logger(), "Unknown command type: %d", 
                       cmd->command_type);
    }
    
    motion_in_progress_ = false;
    
    // Publish motion complete
    auto complete_msg = std::make_unique<std_msgs::msg::Bool>();
    complete_msg->data = true;
    motion_complete_pub_->publish(std::move(complete_msg));
}

void ArmControllerNode::moveToPosition(
    const geometry_msgs::msg::Point& position,
    const geometry_msgs::msg::Quaternion& orientation)
{
    RCLCPP_INFO(this->get_logger(), "Moving to position [%.3f, %.3f, %.3f]",
                position.x, position.y, position.z);
    
    // TODO: Implement actual arm motion using Kinova API
    // 1. Convert position/orientation to joint angles or cartesian pose
    // 2. Send motion command to arm
    // 3. Wait for motion completion
    
    // PLACEHOLDER: Simulate motion
    // Kinova::Api::ConstrainedPose pose;
    // pose.mutable_target_pose()->set_x(position.x);
    // pose.mutable_target_pose()->set_y(position.y);
    // pose.mutable_target_pose()->set_z(position.z);
    // pose.mutable_target_pose()->mutable_theta()->set_x(...); // from quaternion
    // left_arm_base_->ReachPose(pose);
    
    std::this_thread::sleep_for(std::chrono::milliseconds(100)); // Simulate delay
    
    RCLCPP_INFO(this->get_logger(), "Motion completed");
}

void ArmControllerNode::getJointAngles(std::vector<double>& angles) {
    // TODO: Get current joint angles from arm
    // auto feedback = left_arm_cyclic_->RefreshFeedback();
    // for (const auto& actuator : feedback.actuators()) {
    //     angles.push_back(actuator.position() * M_PI / 180.0); // deg to rad
    // }
    
    // PLACEHOLDER: Return dummy angles
    angles = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
}

} // namespace buttoning_system

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<buttoning_system::ArmControllerNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
