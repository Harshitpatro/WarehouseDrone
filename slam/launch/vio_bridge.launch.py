from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Remap RTAB-Map's odometry to MAVROS
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='odom_to_odom_ned',
            arguments=['0', '0', '0', '0', '0', '0', 'odom', 'odom_ned']
        ),
        
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_link_to_base_link_frd',
            arguments=['0', '0', '0', '0', '0', '0', 'base_link', 'base_link_frd']
        ),

        # Custom node to convert odometry to MAVROS format
        Node(
            package='slam',
            executable='odom_converter',
            name='odom_converter',
            parameters=[{
                'input_topic': '/odom',
                'output_topic': '/mavros/odometry/out',
                'frame_id': 'odom_ned',
                'child_frame_id': 'base_link_frd',
                'position_covariance_diagonal': [0.01, 0.01, 0.01],
                'orientation_covariance_diagonal': [0.01, 0.01, 0.01],
                'twist_covariance_diagonal': [0.01, 0.01, 0.01, 0.01, 0.01, 0.01]
            }]
        ),
    ])
