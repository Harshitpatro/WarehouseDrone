from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    return LaunchDescription([
        # Arguments
        DeclareLaunchArgument(
            'use_cuda',
            default_value='true',
            description='Use CUDA acceleration'
        ),
        DeclareLaunchArgument(
            'feature_type',
            default_value='ORB',
            description='Feature detector type: ORB, SIFT, or AKAZE'
        ),
        DeclareLaunchArgument(
            'max_features',
            default_value='2000',
            description='Maximum number of features to detect'
        ),
        
        # Multi-Camera VIO Node
        Node(
            package='slam',
            executable='multi_camera_vio',
            name='multi_camera_vio',
            output='screen',
            parameters=[{
                'use_cuda': LaunchConfiguration('use_cuda'),
                'feature_type': LaunchConfiguration('feature_type'),
                'max_features': LaunchConfiguration('max_features'),
                'scale_factor': 1.2,
                'n_levels': 8,
            }],
            remappings=[
                ('/vision_pose', '/mavros/vision_pose/pose'),
            ]
        ),
        
        # MAVROS Node
       
        
        # Static transform: camera to base_link
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='camera_front_tf',
            arguments=['0.107', '0.0', '-0.044', '0', '0', '0', 'base_link', 'camera_front_link']
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='camera_left_tf',
            arguments=['0', '0.102', '-0.044', '1.5708', '0', '0', 'base_link', 'camera_left_link']
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='camera_right_tf',
            arguments=['0', '-0.102', '-0.044', '-1.5708', '0', '0', 'base_link', 'camera_right_link']
        ),
    ])