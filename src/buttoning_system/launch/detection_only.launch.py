"""Launch file for detection node only (testing)"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'model_path',
            default_value='trainings/onedrv/clothes_button_4_epch500_roi256_lr0.001/',
            description='Path to detection model'
        ),
        
        # Hand detection helper node
        Node(
            package='buttoning_system',
            executable='hand_detection_node',
            name='hand_detection_node',
            output='screen',
        ),
        
        # Main detection node
        Node(
            package='buttoning_system',
            executable='detection_node',
            name='detection_node',
            output='screen',
            parameters=[{
                'max_detections': 10,
                'button_threshold': 0.8,
                'buttonhole_threshold': 0.7,
                'model_path': LaunchConfiguration('model_path'),
            }]
        ),
    ])
