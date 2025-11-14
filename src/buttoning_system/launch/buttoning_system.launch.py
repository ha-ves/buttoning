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
                'model_path': LaunchConfiguration('model_path'),
                'use_sim_time': False,
            }],
            remappings=[
                ('image_raw', 'camera/color/image_raw'),
            ]
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
        
        # RealSense camera node
        Node(
            package='buttoning_system',
            executable='realsense_node',
            name='realsense_node',
            output='screen',
            parameters=[{
                'color_width': 640,
                'color_height': 480,
                'depth_width': 640,
                'depth_height': 480,
                'fps': 30,
                'enable_depth': True,
                'enable_color': True,
                'align_depth_to_color': True,
                'camera_name': 'camera',
                'serial_number': '',
            }]
        ),
    ])
