"""Launch file for the complete buttoning dual-arm system"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    return LaunchDescription([
        # Declare launch arguments
        DeclareLaunchArgument(
            'left_arm_ip',
            default_value='192.168.2.10',
            description='IP address of left Kinova arm'
        ),
        DeclareLaunchArgument(
            'right_arm_ip',
            default_value='192.168.2.12',
            description='IP address of right Kinova arm'
        ),
        DeclareLaunchArgument(
            'model_path',
            default_value='trainings/onedrv/clothes_button_4_epch500_roi256_lr0.001/',
            description='Path to detection model'
        ),
        
        # Hand detection node (helper thread)
        Node(
            package='buttoning_system',
            executable='hand_detection_node',
            name='hand_detection_node',
            output='screen',
            parameters=[{
                'use_sim_time': False,
            }]
        ),
        
        # Detection node (main loop)
        Node(
            package='buttoning_system',
            executable='detection_node',
            name='detection_node',
            output='screen',
            parameters=[{
                'max_detections': 10,
                'button_threshold': 0.8,
                'buttonhole_threshold': 0.7,
                'iou_threshold': 0.01,
                'camera_width': 640,
                'camera_height': 480,
                'camera_fps': 30,
                'model_path': LaunchConfiguration('model_path'),
                'use_sim_time': False,
            }]
        ),
        
        # Arm controller node
        Node(
            package='buttoning_system',
            executable='arm_controller_node',
            name='arm_controller_node',
            output='screen',
            parameters=[{
                'left_arm_ip': LaunchConfiguration('left_arm_ip'),
                'right_arm_ip': LaunchConfiguration('right_arm_ip'),
                'arm_username': 'admin',
                'arm_password': 'admin',
                'use_sim_time': False,
            }]
        ),
        
        # Optional: RealSense camera node (if using librealsense2)
        # Node(
        #     package='realsense2_camera',
        #     executable='realsense2_camera_node',
        #     name='realsense_camera',
        #     output='screen',
        #     parameters=[{
        #         'align_depth.enable': True,
        #         'enable_depth': True,
        #         'enable_color': True,
        #         'depth_module.profile': '640x480x30',
        #         'rgb_camera.profile': '640x480x30',
        #     }]
        # ),
    ])
