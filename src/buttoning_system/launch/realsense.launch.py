"""Launch file for RealSense camera node."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """Generate launch description for RealSense node."""
    
    # Declare launch arguments
    color_width_arg = DeclareLaunchArgument(
        'color_width', default_value='640',
        description='Color image width'
    )
    
    color_height_arg = DeclareLaunchArgument(
        'color_height', default_value='480',
        description='Color image height'
    )
    
    depth_width_arg = DeclareLaunchArgument(
        'depth_width', default_value='640',
        description='Depth image width'
    )
    
    depth_height_arg = DeclareLaunchArgument(
        'depth_height', default_value='480',
        description='Depth image height'
    )
    
    fps_arg = DeclareLaunchArgument(
        'fps', default_value='30',
        description='Frames per second'
    )
    
    enable_depth_arg = DeclareLaunchArgument(
        'enable_depth', default_value='true',
        description='Enable depth stream'
    )
    
    enable_color_arg = DeclareLaunchArgument(
        'enable_color', default_value='true',
        description='Enable color stream'
    )
    
    align_depth_arg = DeclareLaunchArgument(
        'align_depth_to_color', default_value='true',
        description='Align depth to color frames'
    )
    
    camera_name_arg = DeclareLaunchArgument(
        'camera_name', default_value='camera',
        description='Camera namespace'
    )
    
    serial_number_arg = DeclareLaunchArgument(
        'serial_number', default_value='',
        description='Camera serial number (leave empty for any camera)'
    )
    
    # RealSense node
    realsense_node = Node(
        package='buttoning_system',
        executable='realsense_node',
        name='realsense_node',
        output='screen',
        parameters=[{
            'color_width': LaunchConfiguration('color_width'),
            'color_height': LaunchConfiguration('color_height'),
            'depth_width': LaunchConfiguration('depth_width'),
            'depth_height': LaunchConfiguration('depth_height'),
            'fps': LaunchConfiguration('fps'),
            'enable_depth': LaunchConfiguration('enable_depth'),
            'enable_color': LaunchConfiguration('enable_color'),
            'align_depth_to_color': LaunchConfiguration('align_depth_to_color'),
            'camera_name': LaunchConfiguration('camera_name'),
            'serial_number': LaunchConfiguration('serial_number'),
        }]
    )
    
    return LaunchDescription([
        color_width_arg,
        color_height_arg,
        depth_width_arg,
        depth_height_arg,
        fps_arg,
        enable_depth_arg,
        enable_color_arg,
        align_depth_arg,
        camera_name_arg,
        serial_number_arg,
        realsense_node,
    ])
