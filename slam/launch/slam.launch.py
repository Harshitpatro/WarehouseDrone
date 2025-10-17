# Requirements:
#   A realsense D435i camera
#   Flight controller with MAVROS connection
#   Install realsense2 ros2 package (ros-$ROS_DISTRO-realsense2-camera)
#   Install mavros (ros-$ROS_DISTRO-mavros ros-$ROS_DISTRO-mavros-extras)
#
# NOTE: Launch MAVROS separately first:
#   $ ros2 launch mavros apm.launch fcu_url:=/dev/ttyACM0:921600
#
# Then launch this file:
#   $ ros2 launch slam rtabmap.launch.py
#
# Or with custom camera position (example: camera 0.1m forward, 0.05m up from base_link):
#   $ ros2 launch slam rtabmap.launch.py camera_x:=0.1 camera_z:=0.05
#
# IMPORTANT: Adjust camera position parameters based on your robot's setup!

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import Node, SetParameter
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
import math

def generate_launch_description():
    parameters=[{
          'frame_id':'base_link',
          'subscribe_depth':True,
          'subscribe_odom_info':True,
          'approx_sync':True,
          'approx_sync_max_interval': 0.04,  # Reduced from default to handle sync issues
          'wait_imu_to_init':True,
          'always_check_imu_tf':False,      # Added to handle IMU TF warnings
          'RGBD/Enabled': 'True',
        #   'RGBD/ImagesAlreadyRectified': 'True',
        #   'RGBD/LinearUpdate': '0.05',
        #   'RGBD/AngularUpdate': '0.05',
        #   'RGBD/LocalRadius': '30',
        #   'RGBD/OptimizeMaxError': '1.5',
        #   'RGBD/CreateOccupancyGrid': 'True',
        #   'RGBD/NeighborLinkRefining': 'True',
        #   'RGBD/ProximityBySpace': 'True',
        #   'RGBD/ProximityAngle': '120',
        #   'RGBD/ProximityMaxGraphDepth': '150',
        #   'RGBD/ProximityMaxPaths': '7',
        #   'RGBD/MaxOdomCacheSize': '20',
          # Visual Odometry Parameters
          'Vis/FeatureType': '6',  # Using ORB features for Jetson
          'Vis/MaxFeatures': '2000',  # Increased features
          'Vis/MinInliers': '5',    # More permissive
          'Vis/Iterations': '250',   # More iterations
          'Vis/InlierDistance': '0.2', # More permissive distance
          'Vis/CorNNDR': '0.8',     # More permissive matching
          'Vis/MaxDepth': '3.0',    # Increased max depth
          'Vis/MinDepth': '0.0',    # Minimum depth threshold
          # Grid Parameters
          'Grid/RangeMax': '3.0',
          'Grid/CellSize': '0.05',
          'Grid/MaxObstacleHeight': '3.0',
          'Grid/MinClusterSize': '20',
          'Grid/NormalsSegmentation': 'True',
          'Grid/FlatObstacleDetected': 'True',
          'Grid/GroundIsObstacle': 'False',
          'Grid/NoiseFilteringRadius': '0.15',
          'Grid/NoiseFilteringMinNeighbors': '15',
          'Kp/DetectorStrategy': '6',
          # Odometry Parameters
          'Odometry/Strategy': '0',    # Frame-to-Map (more robust)
          'Odometry/MaxFeatures': '2000',   # Increased features
          'Odometry/MinInliers': '5',      # More permissive
          'Odometry/Iterations': '250',     # Balanced iterations
          'Odom/FilteringStrategy': '1',
          'Odom/GuessMotion': 'false',
          'RGBD/NeighborLinkRefining': 'True',
          # Loop Closure and Map Management
          'RGBD/LocalImmunizationRatio': '0.4',  # Increased from default 0.25
          'Mem/LocalSpaceLinksKept': 'true',    # Keep local space links
          'RGBD/LocalLoopDetectionRadius': '10', # Local loop closure radius
          'RGBD/LocalLoopDetectionTime': '10',   # Time window for local loop detection
          'Rtabmap/DetectionRate': '2',     # Increased detection rate
          'Rtabmap/LoopThr': '0.15',        # More permissive loop closure
          'Optimizer/Iterations': '100',     # More iterations for better optimization
          'Optimizer/Strategy': '2',
          'g2o/PixelVariance': '0.3',
          'Vis/EstimationType': '1',         # Added: Use PnP estimation
          # Memory and Performance
          'Mem/STMSize': '30',
          'Mem/RecentWmRatio': '0.4',
          'Rtabmap/TimeThr': '1000'}]

    remappings=[
          ('imu', '/mavros/imu/data'),  # Using MAVROS IMU instead of camera IMU
          ('rgb/image', '/camera/color/image_raw'),
          ('rgb/camera_info', '/camera/color/camera_info'),
          ('depth/image', '/camera/aligned_depth_to_color/image_raw')]

    return LaunchDescription([

        # Launch arguments for camera position relative to base_link
        DeclareLaunchArgument(
            'camera_x', default_value='0.0',
            description='Camera X position from base_link (forward)'),
        DeclareLaunchArgument(
            'camera_y', default_value='0.0',
            description='Camera Y position from base_link (left)'),
        DeclareLaunchArgument(
            'camera_z', default_value='0.0',
            description='Camera Z position from base_link (up)'),
        DeclareLaunchArgument(
            'camera_roll', default_value='0.0',
            description='Camera roll in radians'),
        DeclareLaunchArgument(
            'camera_pitch', default_value='0.0',
            description='Camera pitch in radians'),
        DeclareLaunchArgument(
            'camera_yaw', default_value='0.0',
            description='Camera yaw in radians'),

        # Make sure IR emitter is enabled
        SetParameter(name='depth_module.emitter_enabled', value=1),

        # Static transform from base_link to camera_link
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_to_camera_tf',
            arguments=[
                '0', '0', '0',  # x, y, z
                '0', '0', '0',  # roll, pitch, yaw
                'base_link',
                'camera_link'
            ]
        ),

        # Add proper TF tree structure
        # Map to odom (published by rtabmap)
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='map_to_odom',
            arguments=[
                '0', '0', '0',
                '0', '0', '0',
                'map',
                'odom'
            ]
        ),

        # Odom to base_link (published by rtabmap)
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='odom_to_base',
            arguments=[
                '0', '0', '0',
                '0', '0', '0',
                'odom',
                'base_link'
            ]
        ),

        # Launch camera driver (without IMU)
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([os.path.join(
                get_package_share_directory('realsense2_camera'), 'launch'),
                '/rs_launch.py']),
            launch_arguments={'camera_namespace': '',
                              'enable_gyro': 'false',  # Disabled camera gyro
                              'enable_accel': 'false',  # Disabled camera accel
                              'align_depth.enable': 'true',
                              'enable_sync': 'true',
                              'depth_module.profile': '640x360x30',
                              'rgb_camera.profile': '640x360x30'}.items(),
        ),

        Node(
            package='rtabmap_odom', executable='rgbd_odometry', output='screen',
            parameters=parameters,
            remappings=remappings),

        Node(
            package='rtabmap_slam', executable='rtabmap', output='screen',
            parameters=parameters,
            remappings=remappings,
            arguments=['-d']),

    ])